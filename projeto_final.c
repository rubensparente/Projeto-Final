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

// Parâmetros de calibração do sensor MQ-6
#define RL_VALUE 5.0        // Valor do resistor de carga em kΩ
#define RO_CLEAN_AIR 20.0   // Resistência do sensor em ar limpo (kΩ)

// Limites de concentração de GLP para acionamento de alarmes
#define LIMITE_ALERTA 100   // ppm - nível de alerta (acende LED vermelho)
#define LIMITE_PERIGO 200   // ppm - nível de perigo (pisca mais rápido)
#define LIMITE_CRITICO 300  // ppm - nível crítico (pisca muito rápido)

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
#define THINGSBOARD_ACCESS_TOKEN "earh280znli7s12bf5yv"  // Token de acesso do dispositivo

// Configurações do cliente MQTT
#define MQTT_CLIENT_ID "gas-monitor-pico"  // Identificação do cliente MQTT
#define MQTT_KEEP_ALIVE 60                  // Tempo de keep-alive em segundos
#define TB_TELEMETRY_TOPIC "v1/devices/me/telemetry"  // Tópico para telemetria

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

// Estado do sistema de alarme (compartilhado entre tarefas)
static volatile bool alarme_ativa = false;           // Indica se alarme está ativo
static volatile bool alarme_silenciado = false;      // Indica se alarme foi silenciado
static volatile bool alarme_manual = false;          // Indica se alarme foi acionado manualmente
static volatile float gas_atual = 0;                 // Valor atual da concentração de gás
static volatile uint32_t silencio_inicio = 0;        // Tempo de início do silenciamento
static bool gas_anterior_em_alerta = false;          // Para detectar transições de estado

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

// Função para envio imediato do status de alarme para ThingsBoard
void publish_alarm_status_to_thingsboard(bool alarm_active) {
    // Verifica se há conexão MQTT ativa
    if (!mqtt_connected || mqtt_client == NULL) {
        printf("[TB] Não conectado, pulando envio imediato de alarme\n");
        return;
    }

    // Formata a mensagem JSON com o status do alarme
    snprintf(buffer_mqtt, sizeof(buffer_mqtt),
             "{\"alarm\":%s}", alarm_active ? "true" : "false");

    printf("[TB] Envio imediato alarme: %s\n", buffer_mqtt);

    // Envia a mensagem via MQTT
    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(mqtt_client, TB_TELEMETRY_TOPIC, buffer_mqtt, strlen(buffer_mqtt), 0, 0, NULL, NULL);
    cyw43_arch_lwip_end();

    // Verifica se o envio foi bem sucedido
    if (err == ERR_OK) {
        printf("[TB] Status de alarme enviado com sucesso (imediato)\n");
    } else {
        printf("[TB] Erro ao publicar alarme imediato: %d\n", err);
    }
}

// Função para gerar nome do arquivo de log de sensores com base na data
void get_sensor_filename(char* buffer, size_t size) {
    datetime_t dt;              // Estrutura para armazenar data/hora
    rtc_get_datetime(&dt);      // Obtém data/hora atual do RTC
    // Formata o nome do arquivo: SENSORES_YYYYMMDD.csv
    snprintf(buffer, size, "SENSORES_%04d%02d%02d.csv", dt.year, dt.month, dt.day);
}

// Função para gerar nome do arquivo de log de eventos com base na data
void get_event_filename(char* buffer, size_t size) {
    datetime_t dt;              // Estrutura para armazenar data/hora
    rtc_get_datetime(&dt);      // Obtém data/hora atual do RTC
    // Formata o nome do arquivo: EVENTOS_YYYYMMDD.csv
    snprintf(buffer, size, "EVENTOS_%04d%02d%02d.csv", dt.year, dt.month, dt.day);
}

// Função para verificar se mudou o dia (para criação de novos arquivos de log)
bool check_day_changed(void) {
    datetime_t dt;              // Estrutura para armazenar data/hora
    rtc_get_datetime(&dt);      // Obtém data/hora atual do RTC
    // Calcula um número único representando o dia atual
    uint32_t today = (dt.year * 10000) + (dt.month * 100) + dt.day;

    // Compara com o dia armazenado anteriormente
    if (today != current_day) {
        current_day = today;    // Atualiza o dia atual
        return true;            // Indica que o dia mudou
    }
    return false;               // Dia não mudou
}

// Função para criar cabeçalho do arquivo de sensores
bool create_sensor_header(const char* filename) {
    if (!sd_ready) return false;  // Verifica se SD está pronto

    FIL file;                     // Objeto de arquivo
    // Tenta abrir o arquivo para escrita, criando novo se não existir
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr == FR_OK) {
        // Escreve o cabeçalho com os nomes das colunas
        f_printf(&file, "Data_Hora,Temp_AHT(C),Hum_AHT(%%),");
        f_printf(&file, "Temp_BMP(C),Press_BMP(hPa),Alt_BMP(m),");
        f_printf(&file, "GLP(ppm)\n");
        f_close(&file);          // Fecha o arquivo
        printf("[SD] Arquivo de sensores criado: %s\n", filename);
        return true;             // Indica sucesso
    }
    return (fr == FR_EXIST);     // Retorna true se arquivo já existir
}

// Função para criar cabeçalho do arquivo de eventos
bool create_event_header(const char* filename) {
    if (!sd_ready) return false;  // Verifica se SD está pronto

    FIL file;                     // Objeto de arquivo
    // Tenta abrir o arquivo para escrita, criando novo se não existir
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
    if (fr == FR_OK) {
        // Escreve o cabeçalho com os nomes das colunas
        f_printf(&file, "Data_Hora,Evento,Detalhes,GLP_Atual(ppm),Alarme_Ativo,Manual,Silenciado,MQTT_Enviado\n");
        f_close(&file);          // Fecha o arquivo
        printf("[SD] Arquivo de eventos criado: %s\n", filename);
        return true;             // Indica sucesso
    }
    return (fr == FR_EXIST);     // Retorna true se arquivo já existir
}

// Função para gravar dados dos sensores no cartão SD
bool log_sensores(float temp_aht, float hum, float temp_bmp, float pressure,
                  float altitude, float gas_concentration) {
    if (!sd_ready) return false;  // Verifica se SD está pronto

    // Verifica se mudou o dia (para criar novo arquivo)
    if (check_day_changed()) {
        printf("[SD] Novo dia detectado, criando novos arquivos\n");
    }

    FIL file;                     // Objeto de arquivo
    char filename[32];            // Buffer para nome do arquivo
    get_sensor_filename(filename, sizeof(filename));  // Gera nome do arquivo

    static bool sensor_file_initialized = false;  // Flag de inicialização
    // Cria cabeçalho se for primeiro uso ou se mudou o dia
    if (!sensor_file_initialized || check_day_changed()) {
        create_sensor_header(filename);
        sensor_file_initialized = true;
    }

    // Abre arquivo para escrita em modo append (adiciona ao final)
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("[SD] Erro ao abrir arquivo %s: %d\n", filename, fr);
        return false;             // Retorna erro
    }

    datetime_t dt;                // Estrutura para data/hora
    rtc_get_datetime(&dt);        // Obtém data/hora atual

    // Escreve timestamp formatado
    f_printf(&file, "%04d-%02d-%02d %02d:%02d:%02d,",
             dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    // Escreve dados do AHT10
    f_printf(&file, "%.2f,%.2f,", temp_aht, hum);
    // Escreve dados do BMP280
    f_printf(&file, "%.2f,%.2f,%.2f,", temp_bmp, pressure / 100.0, altitude);
    // Escreve concentração de gás
    f_printf(&file, "%.2f\n", gas_concentration);

    f_close(&file);               // Fecha o arquivo

    printf("[SENSORES] Dados gravados em: %s\n", filename);
    return true;                  // Indica sucesso
}

// Função para gravar eventos no cartão SD
void log_evento(const char* evento, const char* detalhes, bool eh_manual, bool mqtt_enviado) {
    if (!sd_ready) return;        // Verifica se SD está pronto

    FIL file;                     // Objeto de arquivo
    char filename[32];            // Buffer para nome do arquivo
    get_event_filename(filename, sizeof(filename));  // Gera nome do arquivo

    static bool event_file_initialized = false;  // Flag de inicialização
    // Cria cabeçalho se for primeiro uso ou se mudou o dia
    if (!event_file_initialized || check_day_changed()) {
        create_event_header(filename);
        event_file_initialized = true;
    }

    // Abre arquivo para escrita em modo append
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("[SD] Erro ao abrir arquivo de eventos: %d\n", fr);
        return;                   // Sai da função em caso de erro
    }

    datetime_t dt;                // Estrutura para data/hora
    rtc_get_datetime(&dt);        // Obtém data/hora atual

    // Escreve timestamp formatado
    f_printf(&file, "%04d-%02d-%02d %02d:%02d:%02d,",
             dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec);
    // Escreve nome do evento e detalhes
    f_printf(&file, "%s,%s,", evento, detalhes ? detalhes : "");

    // Trata diferença entre eventos manuais e automáticos
    if (eh_manual) {
        f_printf(&file, "N/A,");  // Não aplicável para eventos manuais
    } else {
        f_printf(&file, "%.2f,", gas_atual);  // Escreve valor do gás
    }

    // Escreve status do sistema
    f_printf(&file, "%s,%s,%s,%s\n",
             alarme_ativa ? "SIM" : "NAO",      // Status do alarme
             alarme_manual ? "SIM" : "NAO",     // Se é manual
             alarme_silenciado ? "SIM" : "NAO", // Se está silenciado
             mqtt_enviado ? "SIM" : "NAO");     // Se foi enviado via MQTT

    f_close(&file);               // Fecha o arquivo

    // Imprime log no console
    if (eh_manual) {
        printf("[EVENTO] %s: %s (Evento Manual)\n", evento, detalhes ? detalhes : "");
    } else {
        printf("[EVENTO] %s: %s (Gas: %.2f ppm)\n", evento, detalhes ? detalhes : "", gas_atual);
    }
}

// Função para gerar tom no buzzer com frequência e duração específicas
void buzzer_tone(uint pin, uint frequency, uint duration_ms) {
    if (frequency == 0) {         // Se frequência zero, desliga buzzer
        gpio_put(pin, 0);
        return;
    }

    // Calcula período e meio período em microssegundos
    uint32_t period_us = 1000000 / frequency;
    uint32_t half_period_us = period_us / 2;
    uint32_t end_time = time_us_32() + (duration_ms * 1000);  // Tempo de término

    // Gera onda quadrada para criar o tom
    while (time_us_32() < end_time) {
        gpio_put(pin, 1);         // Liga o pino
        busy_wait_us_32(half_period_us);  // Espera meio período
        gpio_put(pin, 0);         // Desliga o pino
        busy_wait_us_32(half_period_us);  // Espera meio período
    }
}

// Função para alerta sonoro de inicialização do sistema
void alerta_inicializacao(void) {
    printf("[BUZZER] Tocando alerta de inicialização\n");

    // Toca três bipes curtos em dois buzzers com frequências diferentes
    for (int i = 0; i < 3; i++) {
        buzzer_tone(BUZZER1_PIN, 1000, 100);  // Buzzer 1 em 1000Hz
        buzzer_tone(BUZZER2_PIN, 800, 100);   // Buzzer 2 em 800Hz
        sleep_ms(100);                         // Pausa entre bipes
    }

    // Toca um bipe final mais longo
    buzzer_tone(BUZZER1_PIN, 1200, 200);  // Buzzer 1 em 1200Hz
    buzzer_tone(BUZZER2_PIN, 1000, 200);  // Buzzer 2 em 1000Hz

    printf("[BUZZER] Alerta de inicialização concluído\n");
}

// Função de escrita I2C para comunicação com AHT10
int i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    // Escreve bloco de dados no dispositivo I2C
    int result = i2c_write_blocking(i2c0, addr, data, len, false);
    return result < 0 ? -1 : 0;  // Retorna -1 em caso de erro, 0 em sucesso
}

// Função de leitura I2C para comunicação com AHT10
int i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    // Lê bloco de dados do dispositivo I2C
    int result = i2c_read_blocking(i2c0, addr, data, len, false);
    return result < 0 ? -1 : 0;  // Retorna -1 em caso de erro, 0 em sucesso
}

// ========== FUNÇÕES SIMPLIFICADAS DO SD CARD ==========

// Função para inicializar interface SPI para cartão SD
void init_spi_sd(void) {
    spi_init(SPI_PORT, 1000 * 1000);  // Inicializa SPI a 1MHz
    // Configura pinos para função SPI
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    // Configura pino de chip select
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);  // Mantém CS inativo (alto)
}

// Função para inicializar cartão SD
bool init_sd_card(void) {
    printf("[SD] Inicializando SD Card...\n");

    init_spi_sd();  // Inicializa interface SPI

    // Inicializa disco (cartão SD)
    if (disk_initialize(0) != 0) {
        printf("[SD] Erro ao inicializar disco\n");
        return false;  // Falha na inicialização
    }

    // Monta sistema de arquivos
    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("[SD] Erro ao montar FS: %d\n", fr);
        return false;  // Falha ao montar
    }

    printf("[SD] SD Card inicializado com sucesso\n");
    sd_montado = true;  // Atualiza flags
    sd_ready = true;

    // Obtém data atual e armazena para controle de mudança de dia
    datetime_t dt;
    rtc_get_datetime(&dt);
    current_day = (dt.year * 10000) + (dt.month * 100) + dt.day;

    // Cria arquivos de log para o dia atual
    char sensor_file[32], event_file[32];
    get_sensor_filename(sensor_file, sizeof(sensor_file));
    get_event_filename(event_file, sizeof(event_file));

    create_sensor_header(sensor_file);
    create_event_header(event_file);

    return true;  // Indica sucesso
}

// ========== FIM DAS FUNÇÕES DO SD CARD ==========

// Função para inicializar RTC (Real Time Clock)
void init_rtc(void) {
    // Configura data e hora inicial
    datetime_t t = {
        .year = 2026,   // Ano
        .month = 2,     // Mês
        .day = 8,      // Dia
        .dotw = 0,      // Dia da semana (0=domingo)
        .hour = 12,      // Hora
        .min = 29,       // Minuto
        .sec = 0        // Segundo
    };

    rtc_init();          // Inicializa hardware RTC
    rtc_set_datetime(&t); // Define data/hora inicial
}

// Função para inicializar LEDs, botões e buzzers
void init_leds_botao_buzzers(void) {
    // Configura LED verde
    gpio_init(LED_VERDE_PIN);
    gpio_set_dir(LED_VERDE_PIN, GPIO_OUT);
    gpio_put(LED_VERDE_PIN, 0);

    // Configura LED vermelho
    gpio_init(LED_VERMELHO_PIN);
    gpio_set_dir(LED_VERMELHO_PIN, GPIO_OUT);
    gpio_put(LED_VERMELHO_PIN, 0);

    // Configura botão de silenciar alarme
    gpio_init(BOTAO_ALARME_PIN);
    gpio_set_dir(BOTAO_ALARME_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_ALARME_PIN);  // Habilita resistor pull-up interno

    // Configura botão de alarme manual
    gpio_init(BOTAO_MANUAL_PIN);
    gpio_set_dir(BOTAO_MANUAL_PIN, GPIO_IN);
    gpio_pull_up(BOTAO_MANUAL_PIN);  // Habilita resistor pull-up interno

    // Configura buzzer 1
    gpio_init(BUZZER1_PIN);
    gpio_set_dir(BUZZER1_PIN, GPIO_OUT);
    gpio_put(BUZZER1_PIN, 0);

    // Configura buzzer 2
    gpio_init(BUZZER2_PIN);
    gpio_set_dir(BUZZER2_PIN, GPIO_OUT);
    gpio_put(BUZZER2_PIN, 0);
}

// Callback para conexão MQTT
static void mqtt_connection_callback(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao ThingsBoard!\n");
        mqtt_connected = true;  // Atualiza flag de conexão
    } else {
        printf("[MQTT] Falha na conexão MQTT. Código: %d\n", status);
        mqtt_connected = false;  // Atualiza flag de conexão
    }
}

// Callback para resolução DNS
void dns_check_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr != NULL) {
        broker_ip = *ipaddr;  // Armazena endereço IP do broker
        printf("[DNS] Resolvido: %s -> %s\n", name, ipaddr_ntoa(ipaddr));

        // Configura informações de conexão MQTT
        struct mqtt_connect_client_info_t ci = {
            .client_id = MQTT_CLIENT_ID,
            .client_user = THINGSBOARD_ACCESS_TOKEN,  // Usa token como usuário
            .client_pass = NULL,                      // Sem senha
            .keep_alive = MQTT_KEEP_ALIVE,            // Tempo de keep-alive
            .will_topic = NULL,                       // Sem mensagem de will
            .will_msg = NULL,
            .will_qos = 0,
            .will_retain = 0
        };

        printf("[MQTT] Conectando ao broker...\n");
        // Inicia conexão MQTT
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

    // Inicializa hardware WiFi
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return;
    }

    cyw43_arch_enable_sta_mode();  // Configura como estação (cliente WiFi)

    printf("Connecting to Wi-Fi...\n");
    // Conecta à rede WiFi com timeout de 30 segundos
    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return;
    } else {
        printf("Connected.\n");

        // Obtém e exibe endereço IP
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }

    mqtt_client = mqtt_client_new();  // Cria novo cliente MQTT

    // Resolve endereço DNS do ThingsBoard
    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(THINGSBOARD_HOST, &broker_ip, dns_check_callback, NULL);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        // Se já resolvido, chama callback diretamente
        dns_check_callback(THINGSBOARD_HOST, &broker_ip, NULL);
    } else if (err == ERR_INPROGRESS) {
        printf("[DNS] Resolvendo...\n");
    } else {
        printf("[DNS] Erro ao resolver DNS: %d\n", err);
    }
}

// Função para publicar dados no ThingsBoard
void publish_to_thingsboard(float temp_aht, float hum, float temp_bmp, float pressure, float altitude, float gas, bool alarm_active) {
    if (!mqtt_connected || mqtt_client == NULL) {
        printf("[TB] Não conectado, pulando envio\n");
        return;
    }

    // Formata dados em JSON para envio
    snprintf(buffer_mqtt, sizeof(buffer_mqtt),
        "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"altitude\":%.1f,\"gas\":%.2f,\"alarm\":%s}",
        temp_aht, hum, pressure / 100.0, altitude, gas,
        alarm_active ? "true" : "false");

    printf("[TB] Publicando: %s\n", buffer_mqtt);

    // Publica mensagem no tópico de telemetria
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
    // Verifica se sensor está conectado
    i2c_address_t detected = bmp280_check_connected_address();

    if (detected == NONE_DETECTED) {
        return false;  // Sensor não detectado
    }

    // Inicializa sensor
    if (bmp280_init() == NONE_DETECTED) {
        return false;  // Falha na inicialização
    }

    // Configura parâmetros do sensor
    bmp280_set(BMP_I2C_ADDR, MODE_NORMAL, OVERSAMPLING_2X, OVERSAMPLING_16X, T_STANDBY_250MS, FILTER_4X);

    bmp280_initialized = true;  // Atualiza flag
    return true;  // Indica sucesso
}

// Função para ler dados do BMP280
sensors_t read_bmp280(void) {
    sensors_t data = {0};  // Inicializa estrutura de dados

    if (!bmp280_initialized) {
        return data;  // Retorna dados zerados se sensor não inicializado
    }

    data = bmp280_get_all(BMP_I2C_ADDR);  // Lê todos os dados do sensor
    return data;
}

// Função para ler valor bruto do MQ-6
float read_mq6_raw(void) {
    uint16_t adc_value = adc_read();  // Lê valor ADC
    float voltage = adc_value * (3.3f / 4095.0f);  // Converte para tensão

    if (voltage < 0.01) return 0;  // Protege contra divisão por zero

    // Calcula resistência do sensor
    float rs = (3.3f - voltage) / voltage * RL_VALUE;
    return rs;
}

// Função para ler média de várias amostras do MQ-6
float read_mq6_average(int samples) {
    float sum = 0;

    for (int i = 0; i < samples; i++) {
        sum += read_mq6_raw();  // Acumula leituras
        sleep_ms(5);            // Pequena pausa entre leituras
    }

    return sum / samples;  // Retorna média
}

// Função para calibrar sensor MQ-6
float calibrate_mq6(void) {
    float sum = 0;
    int samples = 0;

    // Coleta várias amostras para obter valor médio em ar limpo
    for (int i = 0; i < 20; i++) {
        float rs = read_mq6_raw();
        if (rs > 1.0 && rs < 100.0) {  // Filtra valores fora da faixa esperada
            sum += rs;
            samples++;
        }
        sleep_ms(20);
    }

    if (samples > 5) {
        // Calcula R0 (resistência em ar limpo)
        return (sum / samples) / 10.0;
    }

    return RO_CLEAN_AIR;  // Retorna valor padrão se calibração falhar
}

// Função para calcular concentração de gás a partir da razão RS/RO
float calculate_gas_concentration(float rs_ro_ratio) {
    if (rs_ro_ratio <= 0.1 || rs_ro_ratio > 200) return 0;  // Filtra valores extremos

    // Usa fórmula logarítmica para estimar concentração
    float log_ratio = log10f(rs_ro_ratio);
    float log_ppm = -1.8 * log_ratio + 2.0;
    float ppm = powf(10.0, log_ppm);

    // Limita valores dentro de faixa razoável
    if (ppm < 0.1) ppm = 0;
    if (ppm > 10000) ppm = 10000;

    return ppm;
}

// Função para inicializar sensor MQ-6
void init_mq6(void) {
    adc_init();                     // Inicializa hardware ADC
    adc_gpio_init(MQ6_PIN);         // Configura pino como ADC
    adc_select_input(ADC_CHANNEL);  // Seleciona canal ADC
    mq6_R0 = calibrate_mq6();       // Calibra sensor
}

// Função para exibir dados no display OLED
void display_data(float temp_aht, float hum, float temp_bmp, float pressure, float altitude,
                  float gas_concentration) {
    char line1[32], line2[32], line3[32];  // Buffers para linhas do display
    char sensor_file[16], event_file[16];  // Buffers para nomes de arquivos

    // Formata linha 1: temperatura e umidade do AHT10
    snprintf(line1, sizeof(line1), "T:%.1fC H:%.1f%%", temp_aht, hum);

    // Formata linha 2: pressão e altitude do BMP280
    if (pressure > 0) {
        snprintf(line2, sizeof(line2), "P:%.0fhPa A:%.0fm", pressure / 100.0, altitude);
    } else {
        snprintf(line2, sizeof(line2), "P:N/A A:N/A");  // Se BMP280 não disponível
    }

    // Obtém nomes dos arquivos de log
    get_sensor_filename(sensor_file, sizeof(sensor_file));
    get_event_filename(event_file, sizeof(event_file));

    // Formata status da conexão WiFi/MQTT
    char wifi_status[10];
    if (mqtt_connected) {
        snprintf(wifi_status, sizeof(wifi_status), "TB:OK");
    } else {
        snprintf(wifi_status, sizeof(wifi_status), "WiFi:---");
    }

    // Formata linha 3: status do sistema
    if (alarme_ativa) {
        if (alarme_manual) {
            snprintf(line3, sizeof(line3), "!MANUAL! %s", wifi_status);
        } else {
            snprintf(line3, sizeof(line3), "!GLP:%.2fppm! %s", gas_concentration, wifi_status);
        }
    } else {
        snprintf(line3, sizeof(line3), "GLP:%.2fppm %s", gas_concentration, wifi_status);
    }

    // Limpa display e desenha strings
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, 1, line1);   // Linha 1 na posição Y=0
    ssd1306_draw_string(&oled, 0, 16, 1, line2);  // Linha 2 na posição Y=16
    ssd1306_draw_string(&oled, 0, 32, 1, line3);  // Linha 3 na posição Y=32

    // Exibe status do SD na linha 4 (Y=48)
    if (sd_ready) {
        char log_status[32];
        // Extrai parte da data dos nomes dos arquivos
        snprintf(log_status, sizeof(log_status), "S:%s E:%s",
                 sensor_file + 9, event_file + 8);  // Pula prefixo "SENSORES_" ou "EVENTOS_"
        ssd1306_draw_string(&oled, 0, 48, 1, log_status);
    } else {
        ssd1306_draw_string(&oled, 0, 48, 1, "SD:DESCONECTADO");
    }

    ssd1306_show(&oled);  // Atualiza display
}

// Tarefa FreeRTOS para controlar LEDs e Buzzers com sirene
void led_buzzer_task(void *pvParameters) {
    bool led_state = false;           // Estado atual do LED
    uint32_t last_led_toggle = 0;     // Último tempo de alternância do LED
    uint32_t sirene_start = 0;        // Início do ciclo da sirene
    bool sirene_rising = true;        // Direção da mudança de frequência (subindo/descendo)
    uint32_t current_freq = SIRENE_FREQ_MIN;  // Frequência atual da sirene

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;  // Obtém tempo atual em ms

        // Verifica timeout do silenciamento (reativa automaticamente após 5 minutos)
        if (alarme_silenciado && (now - silencio_inicio > SILENCIO_TIMEOUT_MS)) {
            alarme_silenciado = false;
            log_evento("TIMEOUT_SILENCIO", "Sirene reativada automaticamente", false, mqtt_connected);
        }

        // Se alarme ativo e não silenciado
        if (alarme_ativa && !alarme_silenciado) {
            // Determina intervalo de piscagem baseado na concentração de gás
            uint32_t led_intervalo;
            if (alarme_manual) led_intervalo = 300;                    // Manual: 300ms
            else if (gas_atual >= LIMITE_CRITICO) led_intervalo = 100;  // Crítico: 100ms
            else if (gas_atual >= LIMITE_PERIGO) led_intervalo = 250;  // Perigo: 250ms
            else led_intervalo = 500;                                  // Alerta: 500ms

            // Alterna LED vermelho no intervalo calculado
            if (now - last_led_toggle > led_intervalo) {
                led_state = !led_state;
                gpio_put(LED_VERMELHO_PIN, led_state);
                gpio_put(LED_VERDE_PIN, 0);  // Desliga LED verde
                last_led_toggle = now;
            }

            // Controle da sirene (frequência variável)
            if (now - sirene_start > SIRENE_STEP_MS) {
                // Atualiza frequência (efeito de subida/descida)
                if (sirene_rising) {
                    current_freq += (SIRENE_FREQ_MAX - SIRENE_FREQ_MIN) * SIRENE_STEP_MS / SIRENE_CYCLE_MS;
                    if (current_freq >= SIRENE_FREQ_MAX) {
                        current_freq = SIRENE_FREQ_MAX;
                        sirene_rising = false;  // Inverte direção
                    }
                } else {
                    current_freq -= (SIRENE_FREQ_MAX - SIRENE_FREQ_MIN) * SIRENE_STEP_MS / SIRENE_CYCLE_MS;
                    if (current_freq <= SIRENE_FREQ_MIN) {
                        current_freq = SIRENE_FREQ_MIN;
                        sirene_rising = true;   // Inverte direção
                    }
                }

                // Toca sirene com parâmetros diferentes baseados no nível de alerta
                if (alarme_manual) {
                    // Sirene para alarme manual
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq * 1.2, SIRENE_STEP_MS);
                } else if (gas_atual >= LIMITE_CRITICO) {
                    // Sirene para nível crítico (dois buzzers com frequências diferentes)
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq * 1.5, SIRENE_STEP_MS);
                } else if (gas_atual >= LIMITE_PERIGO) {
                    // Sirene para nível de perigo (dois buzzers na mesma frequência)
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    buzzer_tone(BUZZER2_PIN, current_freq, SIRENE_STEP_MS);
                } else {
                    // Sirene para nível de alerta (apenas um buzzer)
                    buzzer_tone(BUZZER1_PIN, current_freq, SIRENE_STEP_MS);
                    gpio_put(BUZZER2_PIN, 0);  // Desliga segundo buzzer
                }

                sirene_start = now;  // Reinicia contador do ciclo da sirene
            }
        }
        // Se alarme ativo mas silenciado
        else if (alarme_ativa && alarme_silenciado) {
            // Pisca LED verde lentamente (1Hz)
            if (now - last_led_toggle > 1000) {
                led_state = !led_state;
                gpio_put(LED_VERDE_PIN, led_state);
                gpio_put(LED_VERMELHO_PIN, 0);  // Desliga LED vermelho
                last_led_toggle = now;
            }

            // Desliga buzzers
            gpio_put(BUZZER1_PIN, 0);
            gpio_put(BUZZER2_PIN, 0);
        }
        // Se alarme não ativo (sistema normal)
        else {
            // LED verde fixo aceso, desliga vermelho e buzzers
            gpio_put(LED_VERDE_PIN, 1);
            gpio_put(LED_VERMELHO_PIN, 0);
            gpio_put(BUZZER1_PIN, 0);
            gpio_put(BUZZER2_PIN, 0);

            // Reinicia variáveis da sirene
            led_state = false;
            last_led_toggle = now;
            sirene_start = now;
            current_freq = SIRENE_FREQ_MIN;
            sirene_rising = true;
        }

        vTaskDelay(pdMS_TO_TICKS(2));  // Pequeno delay para não sobrecarregar CPU
    }
}

// Tarefa FreeRTOS para monitorar botões
void buttons_task(void *pvParameters) {
    bool last_silence_state = true;  // Estado anterior do botão de silêncio
    bool last_manual_state = true;   // Estado anterior do botão manual
    uint32_t last_press = 0;         // Tempo do último pressionamento

    while (1) {
        // Lê estado atual dos botões (ativo baixo devido a pull-up)
        bool silence_state = gpio_get(BOTAO_ALARME_PIN);
        bool manual_state = gpio_get(BOTAO_MANUAL_PIN);
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Detecção de pressionamento do botão de silenciar (borda de descida)
        if (last_silence_state && !silence_state && (now - last_press > 300)) {
            if (alarme_ativa && !alarme_manual) {  // Só funciona para alarmes automáticos
                alarme_silenciado = !alarme_silenciado;  // Alterna estado
                if (alarme_silenciado) {
                    silencio_inicio = now;  // Registra início do silenciamento
                    log_evento("SILENCIADO", "Alarme silenciado", false, mqtt_connected);
                } else {
                    log_evento("REATIVADO", "Alarme reativado", false, mqtt_connected);
                }
            }
            last_press = now;  // Evita bounce (detecção múltipla)
        }

        // Detecção de pressionamento do botão manual (borda de descida)
        if (last_manual_state && !manual_state && (now - last_press > 300)) {
            if (alarme_ativa && alarme_manual) {
                // Desativa alarme manual
                alarme_ativa = false;
                alarme_manual = false;
                alarme_silenciado = false;
                log_evento("MANUAL_DESATIVADO", "Alarme manual desativado", true, mqtt_connected);
                publish_alarm_status_to_thingsboard(false);  // Envia status imediatamente
            } else {
                // Ativa alarme manual
                alarme_ativa = true;
                alarme_manual = true;
                alarme_silenciado = false;
                gas_atual = LIMITE_PERIGO;  // Define valor simbólico para logs
                log_evento("MANUAL_ATIVADO", "Alarme manual ativado", true, mqtt_connected);
                publish_alarm_status_to_thingsboard(true);   // Envia status imediatamente
            }
            last_press = now;  // Evita bounce
        }

        // Atualiza estados anteriores para detecção de bordas
        last_silence_state = silence_state;
        last_manual_state = manual_state;
        vTaskDelay(pdMS_TO_TICKS(20));  // Delay de 20ms entre verificações
    }
}

// Tarefa principal do sistema
void main_task(void *pvParameters) {
    // Configura funções de comunicação I2C para AHT10
    aht10.iface.i2c_write = i2c_write;
    aht10.iface.i2c_read = i2c_read;
    aht10.iface.delay_ms = sleep_ms;

    init_wifi_mqtt();  // Inicializa WiFi e MQTT

    // Inicializa cartão SD
    if (!init_sd_card()) {
        printf("[SD] Falha ao inicializar SD Card\n");
    } else {
        // Registra evento de inicialização
        log_evento("SISTEMA_INICIADO", "Sistema de monitoramento iniciado (ThingsBoard)", false, mqtt_connected);
    }

    // Inicializa sensores
    if (!init_bmp280()) {
        printf("[BMP280] Sensor não detectado\n");
    }

    init_mq6();  // Inicializa sensor de gás

    if (!AHT10_Init(&aht10)) {
        printf("[AHT10] Sensor não inicializado\n");
    }

    printf("\n=== SISTEMA INICIALIZADO (ThingsBoard) ===\n");
    printf("Arquivos por dia: SENSORES_YYYYMMDD.csv e EVENTOS_YYYYMMDD.csv\n");
    printf("WiFi: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");
    printf("MQTT: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");
    printf("Envio MQTT a cada %d ms (separado da gravação no SD)\n", ENVIO_MQTT_INTERVAL_MS);

    uint32_t leitura_count = 0;       // Contador de leituras
    uint32_t mqtt_publish_count = 0;  // Contador de publicações MQTT

    while (1) {
        float temp_aht = 0, hum = 0;  // Variáveis para AHT10
        sensors_t bmp_data;           // Estrutura para dados BMP280
        float gas_concentration = 0;  // Concentração de gás

        // Lê sensores
        AHT10_ReadTemperatureHumidity(&aht10, &temp_aht, &hum);
        bmp_data = read_bmp280();

        // Lê e calcula concentração de gás
        float rs = read_mq6_average(5);  // Média de 5 leituras
        if (mq6_R0 > 0) {
            gas_concentration = calculate_gas_concentration(rs / mq6_R0);
            gas_atual = gas_concentration;  // Atualiza variável global
        }

        // Controle de alarme automático (se não for manual)
        if (!alarme_manual) {
            bool gas_em_alerta = (gas_concentration >= LIMITE_ALERTA);

            if (gas_em_alerta) {
                bool estava_ativo = alarme_ativa;  // Armazena estado anterior

                if (!alarme_silenciado) {
                    alarme_ativa = true;  // Ativa alarme se não estiver silenciado
                }

                // Se acabou de entrar em estado de alerta, registra evento
                if (!gas_anterior_em_alerta) {
                    char msg[50];
                    snprintf(msg, sizeof(msg), "Gas detectado: %.2f ppm", gas_concentration);
                    log_evento("ALERTA_GAS", msg, false, mqtt_connected);
                }

                // Se alarme acabou de ser ativado, envia status imediatamente
                if (!estava_ativo && alarme_ativa) {
                    publish_alarm_status_to_thingsboard(true);
                }

                gas_anterior_em_alerta = true;  // Atualiza estado anterior
            } else {
                // Se acabou de sair do estado de alerta
                if (gas_anterior_em_alerta) {
                    char msg[50];
                    snprintf(msg, sizeof(msg), "Gas normalizado: %.2f ppm", gas_concentration);
                    log_evento("GAS_NORMALIZADO", msg, false, mqtt_connected);

                    // Desativa alarme e reseta estados
                    alarme_ativa = false;
                    alarme_silenciado = false;
                    gas_atual = 0;

                    // Envia status imediatamente
                    publish_alarm_status_to_thingsboard(false);
                }

                gas_anterior_em_alerta = false;  // Atualiza estado anterior
            }
        }

        absolute_time_t now = get_absolute_time();  // Obtém tempo atual

        // Grava dados dos sensores no SD a cada intervalo configurado
        if (absolute_time_diff_us(last_sensor_log_time, now) / 1000 >= LOG_SENSORES_INTERVAL_MS) {
            leitura_count++;

            printf("\n=== LEITURA #%lu ===\n", leitura_count);
            printf("AHT10: Temp=%.2fC, Hum=%.2f%%\n", temp_aht, hum);

            if (bmp280_initialized) {
                printf("BMP280: Temp=%.2fC, Press=%.2f hPa, Alt=%.2fm\n",
                       bmp_data.temperature, bmp_data.pressure / 100.0, bmp_data.altitude);
            }

            printf("MQ-6: GLP=%.2f ppm\n", gas_concentration);
            printf("Alarme: %s\n", alarme_ativa ? "ATIVO" : "INATIVO");
            printf("WiFi: %s\n", mqtt_connected ? "CONECTADO" : "DESCONECTADO");

            if (sd_ready) {
                if (log_sensores(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                                 bmp_data.altitude, gas_concentration)) {
                    printf("[SENSORES] Dados gravados no SD\n");
                }
            }

            last_sensor_log_time = now;  // Atualiza tempo da última gravação
        }

        // Envia dados via MQTT a cada intervalo configurado
        if (absolute_time_diff_us(last_mqtt_send_time, now) / 1000 >= ENVIO_MQTT_INTERVAL_MS) {
            if (mqtt_connected) {
                publish_to_thingsboard(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                                       bmp_data.altitude, gas_concentration, alarme_ativa);
                mqtt_publish_count++;
                printf("[TB] Dados enviados via ThingsBoard (#%lu)\n", mqtt_publish_count);
            } else {
                printf("[TB] Não conectado, pulando envio\n");
            }

            last_mqtt_send_time = now;  // Atualiza tempo do último envio
        }

        cyw43_arch_poll();  // Processa eventos de rede WiFi

        // Atualiza display com dados atuais
        display_data(temp_aht, hum, bmp_data.temperature, bmp_data.pressure,
                     bmp_data.altitude, gas_concentration);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay de 1 segundo entre ciclos
    }
}

// Função principal (entry point do sistema)
int main() {
    stdio_init_all();   // Inicializa comunicação serial para debugging
    sleep_ms(3000);     // Aguarda estabilização após reset

    printf("\n=== SISTEMA DE MONITORAMENTO AMBIENTAL COM GLP (THINGSBOARD) ===\n");
    printf("=== ENVIO MQTT SEPARADO DA GRAVAÇÃO NO SD + ALERTA IMEDIATO ===\n");

    init_leds_botao_buzzers();  // Inicializa GPIOs

    alerta_inicializacao();     // Toca alerta sonoro de inicialização

    // Inicializa I2C0 para comunicação com AHT10
    i2c_init(i2c0, 100 * 1000);  // 100kHz
    gpio_set_function(AHT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(AHT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(AHT_SDA);      // Habilita pull-up interno
    gpio_pull_up(AHT_SCL);

    // Inicializa I2C1 para comunicação com display OLED
    i2c_init(i2c1, 100 * 1000);  // 100kHz
    gpio_set_function(LCD_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_SDA);      // Habilita pull-up interno
    gpio_pull_up(LCD_SCL);

    init_rtc();  // Inicializa RTC

    // Inicializa display OLED
    ssd1306_init(&oled, 128, 64, LCD_I2C_ADDRESS, i2c1);
    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 10, 10, 1, "Sistema");
    ssd1306_draw_string(&oled, 5, 30, 1, "WiFi+TB GLP");
    ssd1306_show(&oled);

    sleep_ms(2000);  // Exibe mensagem inicial por 2 segundos

    // Cria tarefas FreeRTOS
    xTaskCreate(main_task, "MainTask", 4096, NULL, 1, NULL);
    xTaskCreate(led_buzzer_task, "LedBuzzerTask", 1024, NULL, 2, NULL);
    xTaskCreate(buttons_task, "ButtonsTask", 1024, NULL, 2, NULL);

    gpio_put(LED_VERDE_PIN, 1);  // Acende LED verde indicando sistema pronto
    sleep_ms(1000);

    printf("Sistema pronto.\n");
    printf("Gravação SD: a cada %d ms\n", LOG_SENSORES_INTERVAL_MS);
    printf("Envio MQTT: a cada %d ms (separado) + imediato em alertas\n", ENVIO_MQTT_INTERVAL_MS);

    vTaskStartScheduler();  // Inicia escalonador FreeRTOS

    // Loop infinito (não deve chegar aqui se FreeRTOS estiver rodando)
    while (1) {
        tight_loop_contents();  // Loop de baixo consumo
    }

    return 0;  // Nunca alcançado
}