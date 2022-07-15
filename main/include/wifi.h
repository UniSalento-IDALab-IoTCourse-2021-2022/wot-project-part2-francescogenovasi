#ifndef MESH_NETWORK_SERVER_WIFI_H
#define MESH_NETWORK_SERVER_WIFI_H

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID      "ESP_TEST_OTA"
#define WIFI_PASS      "esptestota"
#define WIFI_MAXIMUM_RETRY  5

static EventGroupHandle_t s_wifi_event_group;

void wifi_init_sta(void);

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


#endif //MESH_NETWORK_SERVER_WIFI_H
