#include "esp_serial.h"

static const char* TAG = "SERIAL";

void init_serial_debug(void) 
{
    uart_config_t uart_config = {
        .baud_rate = 115200,                  // Tốc độ Baud chuẩn để debug
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Khởi hình cấu hình cho UART_NUM_0 (Cổng Log mặc định của ESP-IDF)
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    
    // Gán chính xác chân TX=1 và RX=3 cho bộ UART_NUM_0 qua bộ IO MUX phần cứng
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_TX_NUM, UART_RX_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    esp_vfs_dev_uart_register();
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    esp_log_level_set("*", ESP_LOG_INFO); 

}