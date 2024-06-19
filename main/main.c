#include <stdio.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "inttypes.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "dht.h"
#include "rc522.h"
#include "pcf8574.h"
#include "hd44780.h"

static const char* PIR_TAG = "PIR: ";
static const char* RC522_TAG = "rc522: ";
static rc522_handle_t scanner;

static i2c_dev_t pcf8574;
static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data){
    return pcf8574_port_write(&pcf8574, data);
}

#define SENSOR_BUFFER_SIZE 64
static char sensor_data[SENSOR_BUFFER_SIZE];
static const char *DHT_TAG = "DHT: ";
static float humidity, temperature;

#define SENSOR_TYPE DHT_TYPE_DHT11
#define DHT_GPIO_NUM GPIO_NUM_4

esp_err_t get_sensor_data() {
#ifdef CONFIG_INTERNAL_PULLUP
    gpio_set_pull_mode(DHT_GPIO_NUM, GPIO_PULLUP_ONLY);
#endif
    if (dht_read_float_data(SENSOR_TYPE, DHT_GPIO_NUM, &humidity, &temperature) == ESP_OK) {
        ESP_LOGI(DHT_TAG, "Hum: %.1f Tmp: %.1f", humidity, temperature);
        snprintf(sensor_data, SENSOR_BUFFER_SIZE, "%.1f;%.1f\n", humidity, temperature);
    } else {
        ESP_LOGI(DHT_TAG, "Could not read data from sensor");
        snprintf(sensor_data, SENSOR_BUFFER_SIZE, "Error reading sensor\n");
    }
    return ESP_OK;
}

static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data) {
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void tx_task(void *arg) {
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        get_sensor_data();
        sendData(TX_TASK_TAG, sensor_data);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void rx_task(void *arg) {
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

void display_task(void *pv){
    hd44780_t lcd = {
        .write_cb = write_lcd_data, 
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e  = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3
        }
    };
    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, 0x27, I2C_NUM_0, GPIO_NUM_32, GPIO_NUM_33));

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_switch_backlight(&lcd, true);
    hd44780_clear(&lcd);
    char data[10];
    while(1){
        hd44780_gotoxy(&lcd, 1, 0);
        sprintf(data, "Hum: %.1f", humidity);
        hd44780_puts(&lcd, data);
        hd44780_gotoxy(&lcd, 1, 1);
        sprintf(data, "Tem: %.1f", temperature);
        hd44780_puts(&lcd, data);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void detecting_motion_task(void *pv){
    esp_rom_gpio_pad_select_gpio(2);
    esp_rom_gpio_pad_select_gpio(27);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    gpio_set_level(2,0);
    gpio_set_direction(27, GPIO_MODE_INPUT);
    while (1){
        if(gpio_get_level(27)){
            ESP_LOGI(PIR_TAG, "motion detected\n");
            gpio_set_level(2,1);
            vTaskDelay(1000/portTICK_PERIOD_MS);
            gpio_set_level(2,0);
        }
        else{
            ESP_LOGI(PIR_TAG, "motion not detected\n");
            gpio_set_level(2,0);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;
    esp_rom_gpio_pad_select_gpio(5);
    esp_rom_gpio_pad_select_gpio(15);
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    gpio_set_direction(15, GPIO_MODE_OUTPUT);
    gpio_set_level(5,0);
    gpio_set_level(15,0);
    switch(event_id) {
        case RC522_EVENT_TAG_SCANNED: {
                rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
                ESP_LOGI(RC522_TAG, "Tag scanned (sn: %" PRIu64 ")", tag->serial_number);
                if(tag->serial_number == 99109377392){
                    ESP_LOGI(RC522_TAG, "Authorized tag detected");
                    gpio_set_level(5,1);
                    vTaskDelay(2000/portTICK_PERIOD_MS);
                    gpio_set_level(5,0);
                    ESP_LOGI(RC522_TAG, "LED1 turned off");
                }
                else {
                    ESP_LOGI(RC522_TAG, "Unauthorized tag");
                    gpio_set_level(15,1);
                    vTaskDelay(2000/portTICK_PERIOD_MS);
                    gpio_set_level(15,0);
                    ESP_LOGI(RC522_TAG, "LED2 turned off");
                }
            }
            break;
        default:
            ESP_LOGI(RC522_TAG, "Unhandled event");
            break;
    }
}

void rc522_task(void *pv) {
    rc522_config_t config = {
        .spi.host = VSPI_HOST,
        .spi.miso_gpio = 19,
        .spi.mosi_gpio = 23,
        .spi.sck_gpio = 18,
        .spi.sda_gpio = 21,
    };
    rc522_create(&config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    rc522_start(scanner);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    uart_init();
    //ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, 6 - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, 6 - 2, NULL);
    //xTaskCreate(display_task, "display", 1024 * 4, NULL, configMAX_PRIORITIES - 2, NULL);
    //xTaskCreate(detecting_motion_task, "detecting motion", 1024 * 2, NULL, 6 - 2, NULL);
    //xTaskCreate(rc522_task, "rc522_task", 1024 * 4, NULL, 6 - 1, NULL);
}
