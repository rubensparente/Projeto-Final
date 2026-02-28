// Inclusão de bibliotecas padrão
#include <stdio.h>
#include <string.h>
#include <math.h>

// Inclusão de bibliotecas do Raspberry Pi Pico
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/rtc.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// Inclusão de drivers para sensores e periféricos
#include "aht10.h"
#include "ssd1306.h"
#include "ff.h"
#include "diskio.h"
#include "bmp280.h"

// Inclusão de FreeRTOS para multitarefa
#include "FreeRTOS.h"
#include "task.h"

// Inclusão de bibliotecas para WiFi e MQTT (ThingsBoard)
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"

// Configuração dos pinos para comunicação I2C com AHT10
#define AHT_SDA 0   // Pino de dados I2C para AHT10
#define AHT_SCL 1   // Pino de clock I2C para AHT10
#define AHT_I2C_ADDR 0x38  // Endereço I2C do sensor AHT10

// Configuração dos pinos para display OLED via I2C
#define LCD_SDA 14  // Pino de dados I2C para display OLED
#define LCD_SCL 15  // Pino de clock I2C para display OLED
#define LCD_I2C_ADDRESS 0x3C  // Endereço I2C do display OLED

// Endereço I2C do sensor BMP280
#define BMP_I2C_ADDR 0x76

// Configuração dos pinos para comunicação SPI com cartão SD
#define SPI_PORT spi0     // Porta SPI a ser utilizada
#define PIN_MISO 16       // Pino Master In Slave Out (entrada de dados)
#define PIN_MOSI 19       // Pino Master Out Slave In (saída de dados)
#define PIN_SCK 18        // Pino de clock SPI
#define PIN_CS 17         // Pino de seleção (chip select) para cartão SD

// Configuração do pino para sensor de gás MQ-6
#define MQ6_PIN 28        // Pino analógico do MQ-6 (GPIO 28 = ADC2)
#define ADC_CHANNEL 2     // Canal ADC para GPIO 28

// Pinos para LEDs, botão e buzzers
#define LED_VERDE_PIN 11     // LED verde - indica sistema normal
#define LED_VERMELHO_PIN 13  // LED vermelho - alerta de gás detectado
#define BOTAO_ALARME_PIN 5   // Botão para silenciar/reativar alarme (GPIO 5)
#define BOTAO_MANUAL_PIN 6   // Botão para acionar alarme manualmente (GPIO 6)
#define BUZZER1_PIN 21       // Buzzer 1 (GPIO 21)
#define BUZZER2_PIN 10       // Buzzer 2 (GPIO 10)

// Parâmetros da sirene (efeito sonoro do alarme)
#define SIRENE_FREQ_MIN 300     // Frequência mínima da sirene (Hz)
#define SIRENE_FREQ_MAX 1200    // Frequência máxima da sirene (Hz)
#define SIRENE_CYCLE_MS 2000    // Duração de um ciclo completo da sirene (ms)
#define SIRENE_STEP_MS 10       // Passo de tempo para mudança de frequência (ms)

// ========== PARÂMETROS CORRIGIDOS DO SENSOR MQ-6 ==========
// Parâmetros de calibração do sensor MQ-6 (ajustados)
#define RL_VALUE 5.0        // Valor do resistor de carga em kΩ
#define RO_CLEAN_AIR 20.0   // Resistência do sensor em ar limpo (kΩ) - padrão
#define MIN_VALID_RS 1.0    // Mínimo valor válido de RS (kΩ) - REDUZIDO
#define MAX_VALID_RS 100.0  // Máximo valor válido de RS (kΩ) - AUMENTADO
#define FILTER_ALPHA 0.15   // Fator de suavização (menor = mais suave) - AJUSTADO
#define PREHEAT_TIME_MS 30000 // Tempo de pré-aquecimento: 30 segundos
#define CALIBRATION_SAMPLES 50 // Número de amostras para calibração

// Limites de concentração de GLP para acionamento de alarmes (AUMENTADOS)
#define LIMITE_ALERTA 300   // ppm - nível de alerta
#define LIMITE_PERIGO 600   // ppm - nível de perigo
#define LIMITE_CRITICO 1000 // ppm - nível crítico

// Tempo de timeout para silenciar alarme (5 minutos)
#define SILENCIO_TIMEOUT_MS (5 * 60 * 1000)

// Intervalos de tempo para gravação e envio de dados
#define LOG_SENSORES_INTERVAL_MS 60000  // 1 minuto para gravar sensores no SD
#define ENVIO_MQTT_INTERVAL_MS 60000    // 1 minuto para enviar via MQTT (separado)
#define LOG_EVENTOS_INTERVAL_MS 1000    // Eventos são gravados imediatamente

// Configurações da rede WiFi
#define SSID "brisa-2838103"  // Nome da rede WiFi (SSID)
#define PASS "5wfybzoj"       // Senha da rede WiFi

// Configurações do ThingsBoard Cloud
#define THINGSBOARD_HOST "thingsboard.cloud"  // Endereço do servidor ThingsBoard
#define THINGSBOARD_PORT 1883                 // Porta MQTT padrão
#define THINGSBOARD_ACCESS_TOKEN "BQKB3dIKJeMtFg5WDiH6"  // Token de acesso do dispositivo

// Configurações do cliente MQTT
#define MQTT_CLIENT_ID "gas-monitor-pico"  // Identificação do cliente MQTT
#define MQTT_KEEP_ALIVE 60                  // Tempo de keep-alive em segundos
#define TB_TELEMETRY_TOPIC "v1/devices/me/telemetry"  // Tópico para telemetria

// ========== Enumeração para tipos de alarme (ATUALIZADA) ==========
typedef enum {
    ALARME_NORMAL = 0,           // Estado normal/sem alarme (PADRÃO)
    ALARME_MANUAL = 1,
    ALARME_AUTOMATICO_ALERTA = 2,
    ALARME_AUTOMATICO_PERIGO = 3,
    ALARME_AUTOMATICO_CRITICO = 4
} tipo_alarme_t;

// Variáveis globais para controle do sistema
static ssd1306_t oled;                     // Objeto para controle do display OLED
static FATFS fs;                            // Sistema de arquivos FAT
static bool sd_montado = false;             // Flag indicando se SD foi montado
static bool sd_ready = false;               // Flag indicando se SD está pronto para uso
static absolute_time_t last_sensor_log_time; // Último tempo de gravação de sensores
static absolute_time_t last_mqtt_send_time;  // Último tempo de envio MQTT
static absolute_time_t last_event_log_time;  // Último tempo de gravação de eventos
static bool bmp280_initialized = false;     // Flag de inicialização do BMP280
static float mq6_R0 = RO_CLEAN_AIR;         // Valor de calibração do MQ-6
static AHT10_Handle aht10;                  // Handler para sensor AHT10

// ========== VARIÁVEIS CORRIGIDAS DO MQ-6 ==========
static float gas_filtered = 10.0;           // Inicializa com valor mínimo realista
static bool first_reading = true;           // Controle de primeira leitura

// Estado do sistema de alarme (compartilhado entre tarefas)
static volatile bool alarme_ativa = false;           // Indica se alarme está ativo
static volatile bool alarme_silenciado = false;      // Indica se alarme foi silenciado
static volatile bool alarme_manual = false;          // Indica se alarme foi acionado manualmente
static volatile float gas_atual = 0;                 // Valor atual da concentração de gás
static volatile uint32_t silencio_inicio = 0;        // Tempo de início do silenciamento
static bool gas_anterior_em_alerta = false;          // Para detectar transições de estado

// ========== Variável para armazenar tipo atual de alarme (PADRÃO NORMAL) ==========
static volatile tipo_alarme_t tipo_alarme_atual = ALARME_NORMAL;

// Variável para controle do dia atual (para criação de arquivos diários)
static uint32_t current_day = 0;

// Variáveis para comunicação MQTT com ThingsBoard
static mqtt_client_t *mqtt_client;          // Cliente MQTT
static ip_addr_t broker_ip;                 // Endereço IP do broker MQTT
static bool mqtt_connected = false;         // Flag de conexão MQTT
char buffer_mqtt[256];                      // Buffer para mensagens MQTT

// Protótipos de funções para WiFi e MQTT ThingsBoard
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
void init_wifi_mqtt(void);
void publish_to_thingsboard(float temp_aht, float hum, float temp_bmp, float pressure, float altitude, float gas, bool alarm_active);

// ========== Funções auxiliares para tipo de alarme ==========
const char* get_tipo_alarme_string(void) {
    switch(tipo_alarme_atual) {
        case ALARME_NORMAL: return "NORMAL";
        case ALARME_MANUAL: return "MANUAL";
        case ALARME_AUTOMATICO_ALERTA: return "ALERTA";
        case ALARME_AUTOMATICO_PERIGO: return "PERIGO";
        case ALARME_AUTOMATICO_CRITICO: return "CRITICO";
        default: return "DESCONHECIDO";
    }
}

tipo_alarme_t get_tipo_alarme_atual(void) {
    return tipo_alarme_atual;
}

// ========== Função para definir estado normal (padrão) ==========
void set_estado_normal(void) {
    alarme_ativa = false;
    alarme_manual = false;
    alarme_silenciado = false;
    tipo_alarme_atual = ALARME_NORMAL;
    gas_atual = 0;
    gas_anterior_em_alerta = false;
}

// Função para envio imediato do status de alarme para ThingsBoard
void publish_alarm_status_to_thingsboard(bool alarm_active) {
    if (!mqtt_connected || mqtt_client == NULL) {
        printf("[TB] Não conectado, pulando envio imediato de alarme\n");
        return;
    }

    snprintf(buffer_mqtt, sizeof(buffer_mqtt),
             "{\"alarm\":%s,\"alarm_type\":\"%s\"}", 
             alarm_active ? "true" : "false", 
             get_tipo_alarme_string());

    printf("[TB] Envio imediato alarme: %s\n", buffer_mqtt);

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(mqtt_client, TB_TELEMETRY_TOPIC, buffer_mqtt, strlen(buffer_mqtt), 0, 0, NULL, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        printf("[TB] Status de alarme enviado com sucesso (imediato)\n");
    } else {
        printf("[TB] Erro ao publicar alarme imediato: %d\n", err);
    }
}

// Função para gerar nome do arquivo de log de sensores com base na data
void get_sensor_filename(char* buffer, size_t size) {
    datetime_t dt;
    rtc_get_datetime(&dt);
    snprintf(buffer, size, "SENSORES_%04d%02d%02d.csv", dt.year, dt.month, dt.day);
}

// Função para gerar nome do arquivo de log de eventos com base na data
void get_event_filename(char* buffer, size_t size) {
    datetime_t dt;
    rtc_get_datetime(&dt);
    snprintf(buffer, size, "EVENTOS_%04d%02d%02d.csv", dt.year, dt.month, dt.day);
}

// Função para verificar se mudou o dia (para criação de novos arquivos de log)
bool check_day_changed(void) {
    datetime_t dt;
    rtc_get_datetime(&dt);
    uint32_t today = (dt.year * 10000) + (dt.month * 100) + dt.day;

    if (today != current_day) {
        current_day = today;
        return true;
    }
    return false;
}

// Função para criar cabeçalho do arquivo de sensores
bool create_sensor_header(const char* filename) {
    if (!sd_ready) return false;

    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr == FR_OK) {
        f_printf(&file, "Data_Hora,Temp_AHT(C),Hum_AHT(%%),");
        f_printf(&file, "Temp_BMP(C),Press_BMP(hPa),Alt_BMP(m),");
        f_printf(&file, "GLP(ppm)\n");
        f_close(&file);
        printf("[SD] Arquivo de sensores criado: %s\n", filename);
        return true;
    }
    return (fr == FR_EXIST);
}

// ========== Função para criar cabeçalho do arquivo de eventos (ATUALIZADA) ==========
bool create_event_header(const char* filename) {
    if (!sd_ready) return false;

    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr == FR_OK) {
        f_printf(&file, "Data_Hora,Evento,Detalhes,GLP_Atual(ppm),Alarme_Ativo,Tipo_Alarme,Silenciado,MQTT_Enviado,Origem\n");
        f_close(&file);
        printf("[SD] Arquivo de eventos criado: %s\n", filename);
        return true;
    }
    return (fr == FR_EXIST);
}

// Função para gravar dados dos sensores no cartão SD
bool log_sensores(float temp_aht, float hum, float temp_bmp, float pressure,
                  float altitude, float gas_concentration) {
    if (!sd_ready) return false;

    if (check_day_changed()) {
        printf("[SD] Novo dia detectado, criando novos arquivos\n");
    }

    FIL file;
    char filename[32];
    get_sensor_filename(filename, sizeof(filename));

    static bool sensor_file_initialized = false;
    if (!sensor_file_initialized || check_day_changed()) {
        create_sensor_header(filename);
        sensor_file_initialized = true;
    }

    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("[SD] Erro ao abrir arquivo %s: %d\n", filename, fr);
        return false;
    }

    datetime_t dt;
    rtc_get_datetime(&dt);

    f_printf(&file, "%04d-%02d-%02d %02d:%02d:%02d,",
             dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    f_printf(&file, "%.2f,%.2f,", temp_aht, hum);
    f_printf(&file, "%.2f,%.2f,%.2f,", temp_bmp, pressure / 100.0, altitude);
    f_printf(&file, "%.2f\n", gas_concentration);

    f_close(&file);

    printf("[SENSORES] Dados gravados em: %s\n", filename);
    return true;
}

// ========== Função para gravar eventos no cartão SD (ATUALIZADA) ==========
void log_evento(const char* evento, const char* detalhes, bool eh_manual, 
                bool mqtt_enviado, tipo_alarme_t tipo_alarme) {
    if (!sd_ready) return;

    FIL file;
    char filename[32];
    get_event_filename(filename, sizeof(filename));

    static bool event_file_initialized = false;
    if (!event_file_initialized || check_day_changed()) {
        create_event_header(filename);
        event_file_initialized = true;
    }

    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("[SD] Erro ao abrir arquivo de eventos: %d\n", fr);
        return;
    }

    datetime_t dt;
    rtc_get_datetime(&dt);

    f_printf(&file, "%04d-%02d-%02d %02d:%02d:%02d,",
             dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    f_printf(&file, "%s,%s,", evento, detalhes ? detalhes : "");

    if (eh_manual) {
        f_printf(&file, "N/A,");
    } else {
        f_printf(&file, "%.2f,", gas_atual);
    }

    const char* tipo_str;
    switch(tipo_alarme) {
        case ALARME_NORMAL: tipo_str = "NORMAL"; break;
        case ALARME_MANUAL: tipo_str = "MANUAL"; break;
        case ALARME_AUTOMATICO_ALERTA: tipo_str = "AUTO_ALERTA"; break;
        case ALARME_AUTOMATICO_PERIGO: tipo_str = "AUTO_PERIGO"; break;
        case ALARME_AUTOMATICO_CRITICO: tipo_str = "AUTO_CRITICO"; break;
        default: tipo_str = "DESCONHECIDO"; break;
    }

    f_printf(&file, "%s,%s,%s,%s,%s\n",
             alarme_ativa ? "SIM" : "NAO",
             tipo_str,
             alarme_silenciado ? "SIM" : "NAO",
             mqtt_enviado ? "SIM" : "NAO",
             eh_manual ? "MANUAL" : "AUTOMATICO");

    f_close(&file);

    printf("[EVENTO] %s: %s | Tipo: %s | Gás: %.2f ppm\n", 
           evento, detalhes ? detalhes : "", tipo_str, gas_atual);
}

// Função para gerar tom no buzzer com frequência e duração específicas
void buzzer_tone(uint pin, uint frequency, uint duration_ms) {
    if (frequency == 0) {
        gpio_put(pin, 0);
        return;
    }

    uint32_t period_us = 1000000 / frequency;
    uint32_t half_period_us = period_us / 2;
    uint32_t end_time = time_us_32() + (duration_ms * 1000);

    while (time_us_32() < end_time) {
        gpio_put(pin, 1);
        busy_wait_us_32(half_period_us);
        gpio_put(pin, 0);
        busy_wait_us_32(half_period_us);
    }
}

// Função para alerta sonoro de inicialização do sistema
void alerta_inicializacao(void) {
    printf("[BUZZER] Tocando alerta de inicialização\n");

    for (int i = 0; i < 3; i++) {
        buzzer_tone(BUZZER1_PIN, 1000, 100);
        buzzer_tone(BUZZER2_PIN, 800, 100);
        sleep_ms(100);
    }

    buzzer_tone(BUZZER1_PIN, 1200, 200);
    buzzer_tone(BUZZER2_PIN, 1000, 200);

    printf("[BUZZER] Alerta de inicialização concluído\n");
}

// Função de escrita I2C para comunicação com AHT10
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    int result = i2c_write_blocking(i2c0, addr, data, len, false);
    return result < 0 ? -1 : 0;
}

// Função de leitura I2C para comunicação com AHT10
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    int result = i2c_read_blocking(i2c0, addr, data, len, false);
    return result < 0 ? -1 : 0;
}

// ========== FUNÇÕES SIMPLIFICADAS DO SD CARD ==========

// Função para inicializar interface SPI para cartão SD
void init_spi_sd(void) {
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

// Função para inicializar cartão SD
bool init_sd_card(void) {
    printf("[SD] Inicializando SD Card...\n");

    init_spi_sd();

    if (disk_initialize(0) != 0) {
        printf("[SD] Erro ao inicializar disco\n");
        return false;
    }

    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("[SD] Erro ao montar FS: %d\n", fr);
        return false;
    }

    printf("[SD] SD Card inicializado com sucesso\n");
    sd_montado = true;
    sd_ready = true;

    datetime_t dt;
    rtc_get_datetime(&dt);
    current_day = (dt.year * 10000) + (dt.month * 100) + dt.day;

    char sensor_file[32], event_file[32];
    get_sensor_filename(sensor_file, sizeof(sensor_file));
    get_event_filename(event_file, sizeof(event_file));

    create_sensor_header(sensor_file);
    create_event_header(event_file);

    return true;
}

// ========== FIM DAS FUNÇÕES DO SD CARD ==========

// Função para inicializar RTC (Real Time Clock)
void init_rtc(void) {
    datetime_t t = {
        .year = 2026,
        .month = 2,
        .day = 8,
        .dotw = 0,
        .hour = 12,
        .min = 29,
        .sec = 0
    };

    rtc_init();
    rtc_set_datetime(&t);
}

// Função para inicializar LEDs, botões e buzzers
void init_leds_botao_buzzers(void) {
    gpio_init(LED_VERDE_PIN);
    gpio_set_dir(LED_VERDE_PIN, GPIO_OUT);
    gpio_put(LED_VERDE_PIN, 0);

    gpio_init(LED_VERMELHO_PIN);
    gpio_set_dir(LED_VERMELHO_PIN, GPIO_OUT);
    gpio_put(LED_VERMELHO_PIN, 0);

    gpio_init(BOTAO_ALARME_PIN);
    gpio_set_dir(BOTAO_ALARME_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_ALARME_PIN);

    gpio_init(BOTAO_MANUAL_PIN);
    gpio_set_dir(BOTAO_MANUAL_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_MANUAL_PIN);

    gpio_init(BUZZER1_PIN);
    gpio_set_dir(BUZZER1_PIN, GPIO_OUT);
    gpio_put(BUZZER1_PIN, 0);

    gpio_init(BUZZER2_PIN);
    gpio_set_dir(BUZZER2_PIN, GPIO_OUT);
    gpio_put(BUZZER2_PIN, 0);
}

// Callback para conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao ThingsBoard!\n");
        mqtt_connected = true;
    } else {
        printf("[MQTT] Falha na conexão MQTT. Código: %d\n", status);
        mqtt_connected = false;
    }
}

// Callback para resolução DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr != NULL) {
        broker_ip = *ipaddr;
        printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

        struct mqtt_connect_client_info_t ci = {
            .client_id = MQTT_CLIENT_ID,
            .client_user = THINGSBOARD_ACCESS_TOKEN,
            .client_pass = NULL,
            .keep_alive = MQTT_KEEP_ALIVE,
            .will_topic = NULL,
            .will_msg = NULL,
            .will_qos = 0,
            .will_retain = 0
        };

        printf("[MQTT] Conectando ao broker...\n");
        cyw43_arch_lwip_begin();
        mqtt_client_connect(mqtt_client, &broker_ip, THINGSBOARD_PORT, mqtt_connection_callback, NULL, &ci);
        cyw43_arch_lwip_end();
    } else {
        printf("[DNS] Falha ao resolver DNS para %s\n", name);
    }
}

// Função para inicializar WiFi e MQTT
void init_wifi_mqtt(void) {
    printf("\n=== INICIALIZANDO WIFI E MQTT THINGSBOARD ===\n");

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return;
    } else {
        printf("Connected.\n");

        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    mqtt_client = mqtt_client_new();

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(THINGSBOARD_HOST, &broker_ip, dns_check_callback, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        dns_check_callback(THINGSBOARD_HOST, &broker_ip, NULL);
    } else if (err == ERR_INPROGRESS) {
        printf("[DNS] Resolvendo...\n");
    } else {
        printf("[DNS] Erro ao resolver DNS: %d\n", err);
    }
}

// ========== Função para publicar dados no ThingsBoard (ATUALIZADA) ==========
void publish_to_thingsboard(float temp_aht, float hum, float temp_bmp, float pressure, 
                            float altitude, float gas, bool alarm_active) {
    if (!mqtt_connected || mqtt_client == NULL) {
        printf("[TB] Não conectado, pulando envio\n");
        return;
    }

    snprintf(buffer_mqtt, sizeof(buffer_mqtt),
        "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"altitude\":%.1f,"
        "\"gas\":%.2f,\"alarm\":%s,\"alarm_type\":\"%s\"}",
        temp_aht, hum, pressure / 100.0, altitude, gas,
        alarm_active ? "true" : "false",
        get_tipo_alarme_string());

    printf("[TB] Publicando: %s\n", buffer_mqtt);

    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(mqtt_client, TB_TELEMETRY_TOPIC, buffer_mqtt, strlen(buffer_mqtt), 0, 0, NULL, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        printf("[TB] Publicação enviada com sucesso\n");
    } else {
        printf("[TB] Erro ao publicar: %d\n", err);
    }
}

// Função para inicializar sensor BMP280
bool init_bmp280(void) {
    i2c_address_t detected = bmp280_check_connected_address();

    if (detected == NONE_DETECTED) {
        return false;
    }

    if (bmp280_init() == NONE_DETECTED) {
        return false;
    }

    bmp280_set(BMP_I2C_ADDR, MODE_NORMAL, OVERSAMPLING_2X, OVERSAMPLING_16X, T_STANDBY_250MS, FILTER_4X);

    bmp280_initialized = true;
    return true;
}

// Função para ler dados do BMP280
sensors_t read_bmp280(void) {
    sensors_t data = {0};

    if (!bmp280_initialized) {
        return data;
    }

    data = bmp280_get_all(BMP_I2C_ADDR);
    return data;
}

// ========== FUNÇÕES CORRIGIDAS DO SENSOR MQ-6 ==========

// Função para ler valor bruto do MQ-6 com validação
float read_mq6_raw(void) {
    uint16_t adc_value = adc_read();
    float voltage = adc_value * (3.3f / 4095.0f);
    
    // Proteção contra tensão muito baixa
    if (voltage < 0.01) {
        return MIN_VALID_RS;
    }
    
    float rs = (3.3f - voltage) / voltage * RL_VALUE;
    
    // Limita valores dentro da faixa esperada
    if (rs < MIN_VALID_RS) rs = MIN_VALID_RS;
    if (rs > MAX_VALID_RS) rs = MAX_VALID_RS;
    
    return rs;
}

// Função para ler média de várias amostras do MQ-6
float read_mq6_average(int samples) {
    float sum = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        float rs = read_mq6_raw();
        
        // Aceita valores dentro da faixa esperada
        if (rs >= MIN_VALID_RS && rs <= MAX_VALID_RS) {
            sum += rs;
            valid_samples++;
        }
        
        sleep_ms(10);
    }
    
    if (valid_samples > 0) {
        return sum / valid_samples;
    }
    
    return MIN_VALID_RS;
}

// ========== FUNÇÃO CORRIGIDA - NÃO ZERA MAIS AS LEITURAS ==========
float calculate_gas_concentration(float rs_ro_ratio) {
    // Proteção contra valores inválidos - AGORA MAIS PERMISSIVA
    if (rs_ro_ratio <= 0.001) {  // REDUZIDO de 0.1 para 0.001
        return 10;  // RETORNA 10 ppm em vez de 0
    }
    if (rs_ro_ratio > 1000) {    // AUMENTADO de 200 para 1000
        return 10000;
    }
    
    float log_ratio = log10f(rs_ro_ratio);
    // Coeficientes corrigidos para o MQ-6 (GLP)
    float log_ppm = -1.53f * log_ratio + 1.90f;
    float ppm = powf(10.0f, log_ppm);
    
    // Limita a faixa realista - NÃO ZERA MAIS
    if (ppm < 10) ppm = 10;      // Mínimo: 10 ppm (ar ambiente)
    if (ppm > 10000) ppm = 10000;
    
    return ppm;
}

// ========== NOVA FUNÇÃO - LEITURA COM FILTRO ==========
float read_gas_concentration(void) {
    // Lê média de 15 amostras
    float rs = read_mq6_average(15);
    
    if (mq6_R0 <= 0) {
        mq6_R0 = RO_CLEAN_AIR;
    }
    
    float rs_ro_ratio = rs / mq6_R0;
    float ppm_raw = calculate_gas_concentration(rs_ro_ratio);
    
    // Filtro passa-baixa exponencial
    if (first_reading) {
        gas_filtered = ppm_raw;
        first_reading = false;
    } else {
        gas_filtered = (FILTER_ALPHA * ppm_raw) + ((1.0f - FILTER_ALPHA) * gas_filtered);
    }
    
    // Garante valor mínimo
    if (gas_filtered < 10.0f) {
        gas_filtered = 10.0f;
    }
    
    return gas_filtered;
}

// ========== FUNÇÃO DE CALIBRAÇÃO CORRIGIDA ==========
float calibrate_mq6(void) {
    printf("[MQ-6] Iniciando calibração em ar limpo...\n");
    printf("[MQ-6] Aguarde 30 segundos para estabilização...\n");
    
    float sum = 0;
    int samples = 0;
    
    // Período de aquecimento
    sleep_ms(PREHEAT_TIME_MS);
    
    printf("[MQ-6] Coletando %d amostras...\n", CALIBRATION_SAMPLES);
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float rs = read_mq6_raw();
        
        // Faixa mais ampla para ar limpo
        if (rs > 8.0 && rs < 50.0) {
            sum += rs;
            samples++;
            printf("[MQ-6] Amostra %d: RS = %.2f kΩ (válida)\n", i, rs);
        }
        
        sleep_ms(100);
    }
    
    if (samples > 20) {  // REDUZIDO de 25 para 20
        float rs_media = sum / samples;
        float r0_calculado = rs_media / 10.0f;
        
        printf("[MQ-6] Calibração SUCESSO!\n");
        printf("[MQ-6] RS médio = %.2f kΩ\n", rs_media);
        printf("[MQ-6] R0 calculado = %.2f kΩ\n", r0_calculado);
        printf("[MQ-6] Amostras válidas: %d\n", samples);
        
        return r0_calculado;
    }
    
    printf("[MQ-6] Calibração FALHOU! Usando R0 padrão: %.2f kΩ\n", RO_CLEAN_AIR);
    return RO_CLEAN_AIR;
}

// ========== FUNÇÃO DE INICIALIZAÇÃO CORRIGIDA ==========
void init_mq6(void) {
    adc_init();
    adc_gpio_init(MQ6_PIN);
    adc_select_input(ADC_CHANNEL);
    
    printf("[MQ-6] Sensor inicializado no pino %d\n", MQ6_PIN);
    
    // Reset das variáveis de filtro
    first_reading = true;
    gas_filtered = 10.0;
    
    // Calibração
    mq6_R0 = calibrate_mq6();
}

// Função para exibir dados no display OLED (SEM ALTERAÇÕES - MANTIDA ORIGINAL)
void display_data(float temp_aht, float hum, float temp_bmp, float pressure, float altitude,
                  float gas_concentration) {
    char line1[32], line2[32], line3[32];
    char sensor_file[16], event_file[16];

    snprintf(line1, sizeof(line1), "T:%.1fC H:%.1f%%", temp_aht, hum);

    if (pressure > 0) {
        snprintf(line2, sizeof(line2), "P:%.0fhPa A:%.0fm", pressure / 100.0, altitude);
    } else {
        snprintf(line2, sizeof(line2), "P:N/A A:N/A");
    }

    get_sensor_filename(sensor_file, sizeof(sensor_file));
    get_event_filename(event_file, sizeof(event_file));

    char wifi_status[10];
    if (mqtt_connected) {
        snprintf(wifi_status, sizeof(wifi_status), "TB:OK");
    } else {
        snprintf(wifi_status, sizeof(wifi_status), "WiFi:---");
    }

    if (alarme_ativa) {
        if (alarme_manual) {
            snprintf(line3, sizeof(line3), "!MANUAL! %s", wifi_status);
        } else {
            snprintf(line3, sizeof(line3), "!%s:%.0fppm! %s", 
                    get_tipo_alarme_string(), gas_concentration, wifi_status);
        }
    } else {
        snprintf(line3, sizeof(line3), "GLP:%.0fppm %s", gas_concentration, wifi_status);
    }

    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, 1, line1);
    ssd1306_draw_string(&oled, 0, 16, 1, line2);
    ssd1306_draw_string(&oled, 0, 32, 1, line3);

    if (sd_ready) {
        char log_status[32];
        snprintf(log_status, sizeof(log_status), "S:%s E:%s",
                 sensor_file + 9, event_file + 8);
        ssd1306_draw_string(&oled, 0, 48, 1, log_status);
    } else {
        ssd1306_draw_string(&oled, 0, 48, 1, "SD:DESCONECTADO");
    }

    ssd1306_show(&oled);
}

// Tarefa FreeRTOS para controlar LEDs e Buzzers com sirene
void led_buzzer_task(void *pvParameters) {
    bool led_state = false;
    uint32_t last_led_toggle = 0;
    uint32_t sirene_start = 0;
    bool sirene_rising = true;
    uint32_t current_freq = SIRENE_FREQ_MIN;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (alarme_silenciado && (now - silencio_inicio > SILENCIO_TIMEOUT_MS)) {
            alarme_silenciado = false;
            log_evento("TIMEOUT_SILENCIO", "Sirene reativada automaticamente", 
                      false, mqtt_connected, tipo_alarme_atual);
        }

        if (alarme_ativa && !alarme_silenciado) {
            uint32_t led_intervalo;
            if (alarme_manual) led_intervalo = 300;
            else if (gas_atual >= LIMITE_CRITICO) led_intervalo = 100;
            else if (gas_atual >= LIMITE_PERIGO) led_intervalo = 250;
            else led_intervalo = 500;

            if (now - last_led_toggle > led_intervalo) {
                led_state = !led_state;
                gpio_put(LED_VERMELHO_PIN, led_state);
                gpio_put(LED_VERDE_PIN, 0);
                last_led_toggle = now;
            }

            if (now - sirene_start > SIRENE_STEP_MS) {
                if (sirene_rising) {
                    current_freq += (SIRENE_FREQ_MAX - SIRENE_FREQ_MIN) * SIRENE_STEP_MS / SIRENE_CYCLE_MS;
                    if (current_freq >= SIRENE_FREQ_MAX) {
                        current_freq = SIRENE_FREQ_MAX;
                        sirene_rising = false;
                    }
                } else {
                    current_freq -= (SIRENE_FREQ_MAX - SIRENE_FREQ_MIN) * SIRENE_STEP_MS / SIRENE_CYCLE_MS;
                    if (current_freq <= SIRENE_FREQ_MIN) {
                        current_freq = SIRENE_FREQ_MIN;
                        sirene_rising = true;
                    }
                }

                if (alarme_manual) {
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq * 1.2, SIRENE_STEP_MS);
                } else if (gas_atual >= LIMITE_CRITICO) {
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq * 1.5, SIRENE_STEP_MS);
                } else if (gas_atual >= LIMITE_PERIGO) {
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq, SIRENE_STEP_MS);
                } else {
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    gpio_put(BUZZER2_PIN, 0);
                }

                sirene_start = now;
            }
        }
        else if (alarme_ativa && alarme_silenciado) {
            if (now - last_led_toggle > 1000) {
                led_state = !led_state;
                gpio_put(LED_VERDE_PIN, led_state);
                gpio_put(LED_VERMELHO_PIN, 0);
                last_led_toggle = now;
            }

            gpio_put(BUZZER1_PIN, 0);
            gpio_put(BUZZER2_PIN, 0);
        }
        else {
            gpio_put(LED_VERDE_PIN, 1);
            gpio_put(LED_VERMELHO_PIN, 0);
            gpio_put(BUZZER1_PIN, 0);
            gpio_put(BUZZER2_PIN, 0);

            led_state = false;
            last_led_toggle = now;
            sirene_start = now;
            current_freq = SIRENE_FREQ_MIN;
            sirene_rising = true;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ========== Tarefa de botões (ATUALIZADA) ==========
void buttons_task(void *pvParameters) {
    bool last_silence_state = true;
    bool last_manual_state = true;
    uint32_t last_press = 0;

    while (1) {
        bool silence_state = gpio_get(BOTAO_ALARME_PIN);
        bool manual_state = gpio_get(BOTAO_MANUAL_PIN);
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (last_silence_state && !silence_state && (now - last_press > 300)) {
            if (alarme_ativa && !alarme_manual) {
                alarme_silenciado = !alarme_silenciado;
                if (alarme_silenciado) {
                    silencio_inicio = now;
                    log_evento("SILENCIADO", "Alarme silenciado", false, 
                              mqtt_connected, tipo_alarme_atual);
                } else {
                    log_evento("REATIVADO", "Alarme reativado", false, 
                              mqtt_connected, tipo_alarme_atual);
                }
            }
            last_press = now;
        }

        if (last_manual_state && !manual_state && (now - last_press > 300)) {
            if (alarme_ativa && alarme_manual) {
                set_estado_normal();
                log_evento("MANUAL_DESATIVADO", "Alarme manual desativado - Retorno ao modo NORMAL", 
                          true, mqtt_connected, ALARME_NORMAL);
                publish_alarm_status_to_thingsboard(false);
            } else {
                alarme_ativa = true;
                alarme_manual = true;
                alarme_silenciado = false;
                tipo_alarme_atual = ALARME_MANUAL;
                gas_atual = LIMITE_PERIGO;
                log_evento("MANUAL_ATIVADO", "Alarme manual ativado", 
                          true, mqtt_connected, ALARME_MANUAL);
                publish_alarm_status_to_thingsboard(true);
            }
            last_press = now;
        }

        last_silence_state = silence_state;
        last_manual_state = manual_state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ========== Tarefa principal (ATUALIZADA) ==========
void main_task(void *pvParameters) {
    aht10.iface.i2c_write = i2c_write;
    aht10.iface.i2c_read = i2c_read;
    aht10.iface.delay_ms = sleep_ms;

    init_wifi_mqtt();

    set_estado_normal();

    if (!init_sd_card()) {
        printf("[SD] Falha ao inicializar SD Card\n");
    } else {
        log_evento("SISTEMA_INICIADO", "Sistema de monitoramento iniciado (Modo NORMAL)", 
                  false, mqtt_connected, ALARME_NORMAL);
    }

    if (!init_bmp280()) {
        printf("[BMP280] Sensor não detectado\n");
    }

    // ========== INICIALIZAÇÃO CORRIGIDA DO MQ-6 ==========
    init_mq6();

    if (!AHT10_Init(&aht10)) {
        printf("[AHT10] Sensor não inicializado\n");
    }

    printf("\n=== SISTEMA INICIALIZADO (ThingsBoard) ===\n");
    printf("Modo padrão: NORMAL\n");
    printf("Limites: ALERTA=%dppm PERIGO=%dppm CRITICO=%dppm\n", 
           LIMITE_ALERTA, LIMITE_PERIGO, LIMITE_CRITICO);
    printf("Arquivos por dia: SENSORES_YYYYMMDD.csv e EVENTOS_YYYYMMDD.csv\n");
    printf("WiFi: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");
    printf("MQTT: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");
    printf("Envio MQTT a cada %d ms (separado da gravação no SD)\n", ENVIO_MQTT_INTERVAL_MS);

    uint32_t leitura_count = 0;
    uint32_t mqtt_publish_count = 0;

    while (1) {
        float temp_aht = 0, hum = 0;
        sensors_t bmp_data;
        float gas_concentration = 0;

        AHT10_ReadTemperatureHumidity(&aht10, &temp_aht, &hum);
        bmp_data = read_bmp280();

        // ========== LEITURA CORRIGIDA DO GÁS ==========
        gas_concentration = read_gas_concentration();
        gas_atual = gas_concentration;

        // ========== Controle de alarme automático (ATUALIZADO) ==========
        if (!alarme_manual) {
            bool gas_em_alerta = (gas_concentration >= LIMITE_ALERTA);
            tipo_alarme_t novo_tipo_alarme = tipo_alarme_atual;

            if (gas_em_alerta) {
                bool estava_ativo = alarme_ativa;
                
                if (gas_concentration >= LIMITE_CRITICO) {
                    novo_tipo_alarme = ALARME_AUTOMATICO_CRITICO;
                } else if (gas_concentration >= LIMITE_PERIGO) {
                    novo_tipo_alarme = ALARME_AUTOMATICO_PERIGO;
                } else {
                    novo_tipo_alarme = ALARME_AUTOMATICO_ALERTA;
                }

                if (!alarme_silenciado) {
                    alarme_ativa = true;
                }

                if (novo_tipo_alarme != tipo_alarme_atual) {
                    tipo_alarme_atual = novo_tipo_alarme;
                    char msg[60];
                    snprintf(msg, sizeof(msg), "Nivel %s - Gas: %.2f ppm", 
                            get_tipo_alarme_string(), gas_concentration);
                    log_evento("ALARME_AUTOMATICO", msg, false, mqtt_connected, tipo_alarme_atual);
                }

                if (!gas_anterior_em_alerta) {
                    char msg[50];
                    snprintf(msg, sizeof(msg), "Gas detectado: %.2f ppm", gas_concentration);
                    log_evento("ALERTA_GAS", msg, false, mqtt_connected, tipo_alarme_atual);
                }

                if (!estava_ativo && alarme_ativa) {
                    publish_alarm_status_to_thingsboard(true);
                }

                gas_anterior_em_alerta = true;
            } else {
                if (gas_anterior_em_alerta) {
                    char msg[50];
                    snprintf(msg, sizeof(msg), "Gas normalizado: %.2f ppm", gas_concentration);
                    log_evento("GAS_NORMALIZADO", msg, false, mqtt_connected, tipo_alarme_atual);

                    set_estado_normal();

                    publish_alarm_status_to_thingsboard(false);
                }
                
                gas_anterior_em_alerta = false;
            }
        }

        absolute_time_t now = get_absolute_time();

        if (absolute_time_diff_us(last_sensor_log_time, now) / 1000 >= LOG_SENSORES_INTERVAL_MS) {
            leitura_count++;

            printf("\n=== LEITURA #%lu ===\n", leitura_count);
            printf("AHT10: Temp=%.2fC, Hum=%.2f%%\n", temp_aht, hum);
            printf("Alarme: %s - Tipo: %s\n", 
                   alarme_ativa ? "ATIVO" : "INATIVO", 
                   get_tipo_alarme_string());

            if (bmp280_initialized) {
                printf("BMP280: Temp=%.2fC, Press=%.2f hPa, Alt=%.2fm\n",
                       bmp_data.temperature, bmp_data.pressure / 100.0, bmp_data.altitude);
            }

            printf("MQ-6: GLP=%.2f ppm (R0=%.2f)\n", gas_concentration, mq6_R0);
            printf("WiFi: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");

            if (sd_ready) {
                if (log_sensores(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                                 bmp_data.altitude, gas_concentration)) {
                    printf("[SENSORES] Dados gravados no SD\n");
                }
            }

            last_sensor_log_time = now;
        }

        if (absolute_time_diff_us(last_mqtt_send_time, now) / 1000 >= ENVIO_MQTT_INTERVAL_MS) {
            if (mqtt_connected) {
                publish_to_thingsboard(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                                       bmp_data.altitude, gas_concentration, alarme_ativa);
                mqtt_publish_count++;
                printf("[TB] Dados enviados via ThingsBoard (#%lu)\n", mqtt_publish_count);
            } else {
                printf("[TB] Não conectado, pulando envio\n");
            }

            last_mqtt_send_time = now;
        }

        cyw43_arch_poll();

        display_data(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                     bmp_data.altitude, gas_concentration);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ========== Função principal (ATUALIZADA) ==========
int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n=== SISTEMA DE MONITORAMENTO AMBIENTAL COM GLP (THINGSBOARD) ===\n");
    printf("=== COM IDENTIFICAÇÃO COMPLETA DE TIPO DE ALARME ===\n");
    printf("=== MQ-6 CORRIGIDO - SEM ZERAR LEITURAS ===\n");
    printf("=== TIPOS: NORMAL (PADRÃO) | MANUAL | AUTO_ALERTA | AUTO_PERIGO | AUTO_CRITICO ===\n\n");

    init_leds_botao_buzzers();

    alerta_inicializacao();

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(AHT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(AHT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(AHT_SDA);
    gpio_pull_up(AHT_SCL);

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(LCD_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_SDA);
    gpio_pull_up(LCD_SCL);

    init_rtc();

    set_estado_normal();

    ssd1306_init(&oled, 128, 64, LCD_I2C_ADDRESS, i2c1);
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 10, 10, 1, "Sistema");
    ssd1306_draw_string(&oled, 5, 30, 1, "WiFi+TB GLP");
    ssd1306_show(&oled);

    sleep_ms(2000);

    xTaskCreate(main_task, "MainTask", 4096, NULL, 1, NULL);
    xTaskCreate(led_buzzer_task, "LedBuzzerTask", 1024, NULL, 2, NULL);
    xTaskCreate(buttons_task, "ButtonsTask", 1024, NULL, 2, NULL);

    gpio_put(LED_VERDE_PIN, 1);
    sleep_ms(1000);

    printf("Sistema pronto. Modo: NORMAL (padrão)\n");
    printf("Limites: ALERTA=%dppm PERIGO=%dppm CRITICO=%dppm\n", 
           LIMITE_ALERTA, LIMITE_PERIGO, LIMITE_CRITICO);
    printf("MQ-6: Valor mínimo 10ppm - NÃO ZERA MAIS\n");
    printf("Gravação SD: a cada %d ms\n", LOG_SENSORES_INTERVAL_MS);
    printf("Envio MQTT: a cada %d ms (separado) + imediato em alertas\n", ENVIO_MQTT_INTERVAL_MS);
    printf("Tipos de alarme registrados no arquivo EVENTOS_*.csv: NORMAL, MANUAL, AUTO_ALERTA, AUTO_PERIGO, AUTO_CRITICO\n\n");

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }

    return 0;
}