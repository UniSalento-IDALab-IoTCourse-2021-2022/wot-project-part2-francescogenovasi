#include <nvs_flash.h>
#include "bluetooth_mesh.h"
#include "ble.h"

#define MAIN_TAG "MAIN"


#include "wifi.h"
#include "ota.h"


void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(MAIN_TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    wifi_init_sta();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(MAIN_TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid();

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(MAIN_TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    xTaskCreate(&ota_task, "ota_update_task", 8192, NULL, 5, NULL);
}

