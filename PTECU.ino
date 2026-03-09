#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <esp_task_wdt.h>

// ====================================================================
// 1. PINOS E HARDWARE (PCB Perfeita)
// ====================================================================
// Pinos CAN (Serão atrelados ao SPI Global)
#define CAN_CS   15
#define CAN_SCK  14
#define CAN_MISO 12
#define CAN_MOSI 13

// Pinos SD (Serão atrelados ao VSPI isolado)
#define SD_CS    5
#define SD_SCK   18
#define SD_MISO  19
#define SD_MOSI  23

// Sensores
const int PIN_RPM = 35;
const int PIN_VEL = 32;

// Instâncias
MCP_CAN CAN0(CAN_CS);        // Vai usar o SPI global
SPIClass sdSPI(HSPI);        // Barramento isolado para o SD
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ====================================================================
// 2. ESTRUTURAS E VARIÁVEIS GLOBAIS
// ====================================================================
struct DadosTelemetria {
    uint32_t timestamp;
    float rpm;
    float velocidade;
    float tempCVT;
} estadoAtual;

SemaphoreHandle_t xMutexEstado; 
QueueHandle_t filaSD;
File dataFile;
char nomeArquivo[30];

#define DIAMETRO 0.54f
#define COMPRIMENTO_RODA (3.14159f * DIAMETRO)
#define REDUCAO_FIXA 9.0f
const unsigned long TIMEOUT_US = 500000;
const int DENTES_EIXO_1 = 1;
const int DENTES_EIXO_2 = 3;

volatile unsigned long deltaRPM = 0, deltaVEL = 0;
volatile unsigned long lastISR_RPM = 0, lastISR_VEL = 0;

// ====================================================================
// 3. INTERRUPÇÕES
// ====================================================================
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

void vTaskRPM(void *pvParameters);
void vTaskVelocidade(void *pvParameters);
void vTaskTempCVT(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskCAN(void *pvParameters);

// ====================================================================
// 4. SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n[PTECU] INICIANDO - MODO DUAL SPI BUS");

    xMutexEstado = xSemaphoreCreateMutex();
    filaSD = xQueueCreate(100, sizeof(DadosTelemetria));

    // Desativa ambos os chips fisicamente antes de configurar
    pinMode(CAN_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);
    digitalWrite(SD_CS, HIGH);

    // --- A. BARRAMENTO 1: CAN (Usando objeto SPI Global) ---
    // Mapeia o SPI padrão para os pinos do HSPI (14, 12, 13, 15)
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    
    Serial.print("Iniciando CAN... ");
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println(">>> SUCESSO!");
        CAN0.setMode(MCP_NORMAL);
    } else {
        Serial.println("!!! FALHA !!!");
    }

    // --- B. BARRAMENTO 2: SD CARD (Usando objeto sdSPI isolado) ---
    // Mapeia o VSPI para os pinos do SD (18, 19, 23, 5)
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    Serial.print("Iniciando SD... ");
    // Passamos o sdSPI explicitly para a biblioteca do cartão
    if (!SD.begin(SD_CS, sdSPI, 4000000)) { 
        Serial.println("!!! FALHA OU AUSENTE !!!");
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
            Serial.printf(">>> SUCESSO: Gravando em %s\n", nomeArquivo);
        }
    }

    // --- C. SENSORES ---
    pinMode(PIN_RPM, INPUT); 
    pinMode(PIN_VEL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), isrRPM, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_VEL), isrVEL, FALLING);
    Wire.begin(21, 22);
    mlx.begin();

    esp_task_wdt_config_t twdt_config = { .timeout_ms = 8000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
    esp_task_wdt_reconfigure(&twdt_config);

    // --- D. TASKS ---
    xTaskCreatePinnedToCore(vTaskRPM,         "RPM", 3072, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskVelocidade, "VEL", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskTempCVT,     "CVT", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD,          "SD",  4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskCAN,         "CAN", 4096, NULL, 5, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// ====================================================================
// 5. TASKS E LÓGICA
// ====================================================================
void enviarMsgCAN(uint32_t id, float valor) {
    int16_t valorInt = (int16_t)(valor * 100.0f);
    if (id == 0x200) valorInt = (int16_t)valor;

    byte data[2] = { (byte)(valorInt >> 8), (byte)(valorInt & 0xFF) };
    
    // A biblioteca MCP_CAN vai usar o SPI global que mapeamos para o HSPI
    byte sndStat = CAN0.sendMsgBuf(id, 0, 2, data);

    if(sndStat != CAN_OK) {
        // Opcional: Descomente para ver erros de fiação (Stat 6)
         Serial.printf("[!] Erro CAN ID 0x%X: Stat %d\n", id, sndStat);
        if(sndStat == 6) CAN0.setMode(MCP_NORMAL); 
    }
}

void vTaskCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        DadosTelemetria p = estadoAtual;
        xSemaphoreGive(xMutexEstado);
        
        enviarMsgCAN(0x200, p.rpm);
        enviarMsgCAN(0x201, p.velocidade);
        enviarMsgCAN(0x202, p.tempCVT);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void vTaskSD(void *pvParameters) {
    DadosTelemetria p;
    int contadorFlush = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &p, portMAX_DELAY)) {
            if (dataFile) {
                // A gravação usará o sdSPI (VSPI) isolado
                dataFile.printf("%u;%.0f;%.1f;%.1f\n", p.timestamp, p.rpm, p.velocidade, p.tempCVT);
                if (++contadorFlush >= 20) { 
                    dataFile.flush(); 
                    contadorFlush = 0; 
                }
            }
        }
    }
}

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
        vTaskDelay(pdMS_TO_TICKS(25));
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
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void vTaskTempCVT(void *pvParameters) {
    for (;;) {
        float temp = mlx.readObjectTempC();
        if (isnan(temp)) temp = -99.9;
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.tempCVT = temp;
        xSemaphoreGive(xMutexEstado);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
