#ifndef __ESP_SERIAL_H_
#define __ESP_SERIAL_H_

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/uart.h" // Thêm thư viện điều khiển UART
#include "esp_vfs_dev.h" // Thêm thư viện này để map stdout sang UART

#define UART_TX_NUM        1
#define UART_RX_NUM        3
#define UART_BUF_SIZE      1024


void init_serial_debug(void);

#endif