#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <esp_task_wdt.h>

// --- 1. CONFIGURAÇÃO DE TAXAS ---
#define TAXA_RPM_MS        25   
#define TAXA_VELOCIDADE_MS 25   
#define TAXA_TEMP_CVT_MS   200  
#define TAXA_ENVIO_CAN_MS  25   

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
char nomeArquivo[30];

// --- 2. PINOS E HARDWARE ---
const int PIN_RPM = 35;
const int PIN_VEL = 32;

// CAN (HSPI)
#define CAN_CS 15
#define CAN_SCK 14
#define CAN_MISO 12
#define CAN_MOSI 13
MCP_CAN CAN0(CAN_CS);

// SD CARD (VSPI)
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
SPIClass sdSPI(VSPI);

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

const uint32_t ID_RPM = 0x200;
const uint32_t ID_VEL = 0x201;
const uint32_t ID_TEMP = 0x202;

// --- 3. ISRs ---
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

// Protótipos
void vTaskRPM(void *pvParameters);
void vTaskVelocidade(void *pvParameters);
void vTaskTempCVT(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskCAN(void *pvParameters);
void enviarMsgCAN(long id, float valor);

// ====================================================================
// SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n [PTECU] INICIALIZANDO - MODO ESTÁVEL ");

    xMutexEstado = xSemaphoreCreateMutex();
    filaSD = xQueueCreate(100, sizeof(DadosTelemetria));

    // 1. CAN PRIMEIRO (Prioridade máxima de hardware)
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(" !!! ERRO CRÍTICO: CAN NÃO INICIOU !!!");
    } else {
        CAN0.setMode(MCP_NORMAL);
        Serial.println(" >>> CAN OK (500k / 8MHz)");
    }

    // 2. SENSORES E I2C
    pinMode(PIN_RPM, INPUT); 
    pinMode(PIN_VEL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_VEL), isrVEL, FALLING);
    Wire.begin(21, 22);
    mlx.begin();

    // 3. SD CARD (Instância VSPI isolada)
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println(" !!! AVISO: SD CARD FALHOU (CAN continuará operando) !!!");
    } else {
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
            Serial.printf(" >>> SD OK: %s\n", nomeArquivo);
        }
    }

    // 4. WATCHDOG
    esp_task_wdt_config_t twdt_config = { .timeout_ms = 8000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
    esp_task_wdt_reconfigure(&twdt_config);

    // 5. TASKS (Core 1 para CAN, Core 0 para Sensores/SD)
    xTaskCreatePinnedToCore(vTaskRPM,         "RPM", 3072, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskVelocidade, "VEL", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskTempCVT,     "CVT", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD,          "SD",  4096, NULL, 1, NULL, 0); // Prioridade baixa para o SD
    xTaskCreatePinnedToCore(vTaskCAN,         "CAN", 4096, NULL, 5, NULL, 1); // Prioridade máxima no Core 1
}

void loop() { vTaskDelete(NULL); }

// ====================================================================
// TASK CAN COM DEBUG
// ====================================================================
void vTaskCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        DadosTelemetria p = estadoAtual;
        xSemaphoreGive(xMutexEstado);
        
        // Envia e verifica status
        enviarMsgCAN(ID_RPM, p.rpm);
        enviarMsgCAN(ID_VEL, p.velocidade);
        enviarMsgCAN(ID_TEMP, p.tempCVT);

        vTaskDelay(pdMS_TO_TICKS(TAXA_ENVIO_CAN_MS));
    }
}

void enviarMsgCAN(long id, float valor) {
    int16_t valorInt = (int16_t)(valor * 100.0f);
    byte data[2] = { (byte)(valorInt >> 8), (byte)(valorInt & 0xFF) };
    
    byte sndStat = CAN0.sendMsgBuf(id, 0, 2, data);
    
    if(sndStat != CAN_OK) {
        // Se der erro 6 ou 7, o buffer do MCP2515 está cheio (barramento congestionado ou sem terminação)
        Serial.printf(" [!] Erro CAN ID 0x%X: Stat %d\n", id, sndStat);
    }
}

// ====================================================================
// OUTRAS TASKS (RPM, VEL, CVT, SD)
// ====================================================================

void vTaskRPM(void *pvParameters) {
    for (;;) {
        unsigned long agora_us = micros();
        unsigned long dR, lR;
        noInterrupts();
        dR = deltaRPM; lR = lastISR_RPM;
        interrupts();

        float calc_rpm = (agora_us - lR > TIMEOUT_US) ? 0 : (60000000.0f / (dR * DENTES_EIXO_1));

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.timestamp = millis();
        estadoAtual.rpm = calc_rpm;
        xQueueSend(filaSD, &estadoAtual, 0);
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_RPM_MS));
    }
}

void vTaskVelocidade(void *pvParameters) {
    for (;;) {
        unsigned long agora_us = micros();
        unsigned long dV, lV;
        noInterrupts();
        dV = deltaVEL; lV = lastISR_VEL;
        interrupts();

        float calc_vel = (agora_us - lV > TIMEOUT_US) ? 0 : ((1000000.0f * COMPRIMENTO_RODA * 3.6f) / (dV * REDUCAO_FIXA * DENTES_EIXO_2));

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.velocidade = calc_vel;
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_VELOCIDADE_MS));
    }
}

void vTaskTempCVT(void *pvParameters) {
    for (;;) {
        float temp = mlx.readObjectTempC();
        if (isnan(temp)) temp = -99.9;
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.tempCVT = temp;
        xSemaphoreGive(xMutexEstado);
        vTaskDelay(pdMS_TO_TICKS(TAXA_TEMP_CVT_MS));
    }
}

void vTaskSD(void *pvParameters) {
    DadosTelemetria p;
    int counter = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &p, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%.0f;%.1f;%.1f\n", p.timestamp, p.rpm, p.velocidade, p.tempCVT);
                if (++counter >= 20) { // Reduzi para 20 para flushes mais frequentes e rápidos
                    dataFile.flush(); 
                    counter = 0; 
                }
            }
        }
    }
}
