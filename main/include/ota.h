#ifndef MESH_NETWORK_SERVER_OTA_H
#define MESH_NETWORK_SERVER_OTA_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "errno.h"
#include "esp_wifi.h"

#define OTA_UPDATE_URL "http://fridaycloud.ddns.net:8000/mesh_network_beacon.bin"
#define OTA_TIMEOUT_MS 300

void ota_task(void *pvParameter);

#endif //MESH_NETWORK_SERVER_OTA_H
