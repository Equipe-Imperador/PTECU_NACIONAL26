#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <esp_task_wdt.h>

// --- 1. CONFIGURAÇÃO DE TAXAS (em Milissegundos) ---
#define TAXA_RPM_MS        25   // 20ms = 40Hz 8 pontos por frenagem 
#define TAXA_VELOCIDADE_MS 25   // 20ms = 40Hz
#define TAXA_TEMP_CVT_MS   200  // 200ms = 5Hz
#define TAXA_ENVIO_CAN_MS  25   // 40Hz (Acompanha a atualização rápida)

// --- 2. Estrutura de Dados e Sincronismo ---
struct DadosTelemetria {
    uint32_t timestamp;
    float rpm;
    float velocidade;
    float tempCVT;
};

DadosTelemetria estadoAtual;
SemaphoreHandle_t xMutexEstado; 
QueueHandle_t filaSD;
File dataFile;
char nomeArquivo[20];

// --- 3. Pinos e Hardware ---
const int PIN_RPM = 35;
const int PIN_VEL = 32;

// SPI CAN (HSPI)
#define CAN_CS 15
#define CAN_SCK 14
#define CAN_MISO 12
#define CAN_MOSI 13
SPIClass SPI_CAN(HSPI);
MCP_CAN CAN0(CAN_CS);

// SPI SD (VSPI padrão)
#define SD_CS 5

// I2C para MLX90614
#define I2C_SDA 21
#define I2C_SCL 22
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Constantes Físicas
#define DIAMETRO 0.54f
#define COMPRIMENTO_RODA (3.14159f * DIAMETRO)
#define REDUCAO_FIXA 9.0f
const unsigned long TIMEOUT_US = 500000;
const int DENTES_EIXO_1 = 1;
const int DENTES_EIXO_2 = 3;

// Variáveis de Interrupção
volatile unsigned long deltaRPM = 0, deltaVEL = 0;
volatile unsigned long lastISR_RPM = 0, lastISR_VEL = 0;

// IDs CAN
const uint32_t ID_RPM = 0x200;
const uint32_t ID_VEL = 0x201;
const uint32_t ID_TEMP = 0x202;

// --- 4. ISRs (Interrupções) ---
void IRAM_ATTR isrRPM() {
    unsigned long agora = micros();
    if (agora - lastISR_RPM > 11765) { 
        deltaRPM = agora - lastISR_RPM;
        lastISR_RPM = agora;
    }
}

void IRAM_ATTR isrVEL() {
    unsigned long agora = micros();
    if (agora - lastISR_VEL > 1000) { 
        deltaVEL = agora - lastISR_VEL;
        lastISR_VEL = agora;
    }
}

// --- 5. Protótipos das Tasks ---
void vTaskRPM(void *pvParameters);         // 50Hz
void vTaskVelocidade(void *pvParameters);  // 50Hz
void vTaskTempCVT(void *pvParameters);     // 5Hz
void vTaskSD(void *pvParameters);
void vTaskCAN(void *pvParameters);
void enviarMsgCAN(long id, float valor);

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mlx.begin()) {
        Serial.println("Erro: Sensor MLX90614 nao detectado!");
    }

    pinMode(PIN_RPM, INPUT_PULLUP);
    pinMode(PIN_VEL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_VEL), isrVEL, FALLING);

    SPI_CAN.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(500); }
    CAN0.setMode(MCP_NORMAL);

    if (SD.begin(SD_CS)) {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/PTECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;rpm;vel;temp_cvt");
            dataFile.flush();
        }
    }

    // Inicializa Mutex e Fila
    xMutexEstado = xSemaphoreCreateMutex();
    filaSD = xQueueCreate(100, sizeof(DadosTelemetria)); // Aumentado para 100 pois 50Hz enche rápido

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);

    // --- Criação das Tasks ---
    // Core 0: Leituras Matemáticas
    xTaskCreatePinnedToCore(vTaskRPM,        "RPM", 3072, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskVelocidade, "VEL", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskTempCVT,    "CVT", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD,         "SD",  4096, NULL, 1, NULL, 0);

    // Core 1: CAN
    xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// -------------------------------------------------------------------
// TAREFA 1: RPM (50Hz)
// -------------------------------------------------------------------
void vTaskRPM(void *pvParameters) {
    for (;;) {
        unsigned long agora_us = micros();
        unsigned long dR, lR;

        // Cópia rápida das variáveis para não travar a interrupção
        noInterrupts();
        dR = deltaRPM; 
        lR = lastISR_RPM;
        interrupts();

        float calc_rpm = (agora_us - lR > TIMEOUT_US) ? 0 : (60000000.0f / (dR * DENTES_EIXO_1));

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.timestamp = millis();
        estadoAtual.rpm = calc_rpm;
        
        // A tarefa RPM "empurra" o estado global para a fila do SD (50 vezes por segundo)
        xQueueSend(filaSD, &estadoAtual, 0);
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_RPM_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 2: VELOCIDADE (50Hz)
// -------------------------------------------------------------------
void vTaskVelocidade(void *pvParameters) {
    for (;;) {
        unsigned long agora_us = micros();
        unsigned long dV, lV;

        noInterrupts();
        dV = deltaVEL; 
        lV = lastISR_VEL;
        interrupts();

        float calc_vel = (agora_us - lV > TIMEOUT_US) ? 0 : ((1000000.0f * COMPRIMENTO_RODA * 3.6f) / (dV * REDUCAO_FIXA * DENTES_EIXO_2));

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.velocidade = calc_vel;
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_VELOCIDADE_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 3: TEMPERATURA CVT (5Hz)
// -------------------------------------------------------------------
void vTaskTempCVT(void *pvParameters) {
    for (;;) {
        // Leitura do I2C (demora um pouquinho mais, por isso roda a 5Hz separado)
        float temp = mlx.readObjectTempC();
        if (isnan(temp)) temp = -99.9;

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.tempCVT = temp;
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_TEMP_CVT_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 4: SD DATALOGGER
// -------------------------------------------------------------------
void vTaskSD(void *pvParameters) {
    DadosTelemetria p;
    int counter = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &p, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%.0f;%.1f;%.1f\n", p.timestamp, p.rpm, p.velocidade, p.tempCVT);
                
                // Flush a cada 40 gravações = 1 vez por segundo
                if (++counter >= 40) { 
                    dataFile.flush(); 
                    counter = 0; 
                }
            }
        }
    }
}

// -------------------------------------------------------------------
// TAREFA 5: CAN (50Hz)
// -------------------------------------------------------------------
void vTaskCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        DadosTelemetria p = estadoAtual;
        xSemaphoreGive(xMutexEstado);
        
        enviarMsgCAN(ID_RPM, p.rpm);
        enviarMsgCAN(ID_VEL, p.velocidade);
        enviarMsgCAN(ID_TEMP, p.tempCVT);

        vTaskDelay(pdMS_TO_TICKS(TAXA_ENVIO_CAN_MS));
    }
}

void enviarMsgCAN(long id, float valor) {
    int16_t valorInt = (int16_t)(valor * 100.0f);
    byte data[2] = { (byte)(valorInt >> 8), (byte)(valorInt & 0xFF) };
    CAN0.sendMsgBuf(id, 0, 2, data);
}
