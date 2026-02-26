#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <esp_task_wdt.h>

// --- 1. Estruturas e Filas ---
struct DadosTelemetria {
    uint32_t timestamp;
    float rpm;
    float velocidade;
    float tempCVT;
};

QueueHandle_t filaCAN;
QueueHandle_t filaSD;
File dataFile;
char nomeArquivo[20];

// --- 2. Pinos e Hardware ---
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

// --- 3. ISRs (Interrupções) ---
void IRAM_ATTR isrRPM() {
    unsigned long agora = micros();
    if (agora - lastISR_RPM > 11765) { // Debounce RPM
        deltaRPM = agora - lastISR_RPM;
        lastISR_RPM = agora;
    }
}

void IRAM_ATTR isrVEL() {
    unsigned long agora = micros();
    if (agora - lastISR_VEL > 1000) { // Debounce Velocidade
        deltaVEL = agora - lastISR_VEL;
        lastISR_VEL = agora;
    }
}

// --- 4. Protótipos ---
void vTaskCalculo(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskCAN(void *pvParameters);
void enviarMsgCAN(long id, float valor);

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Inicializa I2C nos pinos 21 e 22
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mlx.begin()) {
        Serial.println("Erro: Sensor MLX90614 nao detectado!");
    }

    pinMode(PIN_RPM, INPUT_PULLUP);
    pinMode(PIN_VEL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_VEL), isrVEL, FALLING);

    // CAN (HSPI)
    SPI_CAN.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(500); }
    CAN0.setMode(MCP_NORMAL);

    // SD Datalogger
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

    // Watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);

    filaCAN = xQueueCreate(10, sizeof(DadosTelemetria));
    filaSD = xQueueCreate(30, sizeof(DadosTelemetria));

    if (filaCAN && filaSD) {
        xTaskCreatePinnedToCore(vTaskCalculo, "Calc", 4096, NULL, 3, NULL, 0);
        xTaskCreatePinnedToCore(vTaskSD, "SD", 4096, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 2, NULL, 1);
    }
}

void loop() { vTaskDelete(NULL); }

// --- TAREFA CALCULO (Core 0) ---
void vTaskCalculo(void *pvParameters) {
    DadosTelemetria p;
    for (;;) {
        unsigned long agora_us = micros();
        unsigned long dR, dV, lR, lV;

        // Cópia segura das variáveis da interrupção
        noInterrupts();
        dR = deltaRPM; dV = deltaVEL;
        lR = lastISR_RPM; lV = lastISR_VEL;
        interrupts();

        p.timestamp = millis();
        
        // Cálculos de frequência
        p.rpm = (agora_us - lR > TIMEOUT_US) ? 0 : (60000000.0f / (dR * DENTES_EIXO_1));
        p.velocidade = (agora_us - lV > TIMEOUT_US) ? 0 : ((1000000.0f * COMPRIMENTO_RODA * 3.6f) / (dV * REDUCAO_FIXA * DENTES_EIXO_2));
        
        // Leitura I2C (CVT)
        p.tempCVT = mlx.readObjectTempC();
        if (isnan(p.tempCVT)) p.tempCVT = -99.9;

        xQueueSend(filaCAN, &p, 0);
        xQueueSend(filaSD, &p, 0);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// --- TAREFA SD (Core 0) ---
void vTaskSD(void *pvParameters) {
    DadosTelemetria p;
    int counter = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &p, portMAX_DELAY) == pdPASS) {
            if (dataFile) {
                dataFile.printf("%u;%.0f;%.1f;%.1f\n", p.timestamp, p.rpm, p.velocidade, p.tempCVT);
                if (++counter >= 50) { dataFile.flush(); counter = 0; }
            }
        }
    }
}

// --- TAREFA CAN (Core 1) ---
void vTaskCAN(void *pvParameters) {
    DadosTelemetria p;
    esp_task_wdt_add(NULL);
    for (;;) {
        if (xQueueReceive(filaCAN, &p, portMAX_DELAY)) {
            esp_task_wdt_reset();
            
            // Envio CAN conforme IDs definidos
            enviarMsgCAN(ID_RPM, p.rpm);
            enviarMsgCAN(ID_VEL, p.velocidade);
            enviarMsgCAN(ID_TEMP, p.tempCVT);
        }
    }
}

void enviarMsgCAN(long id, float valor) {
    int16_t valorInt = (int16_t)(valor * 100.0f);
    byte data[2] = { (byte)(valorInt >> 8), (byte)(valorInt & 0xFF) };
    CAN0.sendMsgBuf(id, 0, 2, data);
}
