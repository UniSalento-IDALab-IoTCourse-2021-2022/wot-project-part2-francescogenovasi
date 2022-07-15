#ifndef TEST_MESH_NETWORK_BLUETOOTH_MESH_H
#define TEST_MESH_NETWORK_BLUETOOTH_MESH_H

#include <esp_ble_mesh_ble_api.h>
#include <esp_ble_mesh_provisioning_api.h>
#include <esp_ble_mesh_config_model_api.h>
#include <esp_ble_mesh_generic_model_api.h>
#include <esp_ble_mesh_common_api.h>
#include <string.h>
#include <esp_bt_device.h>
#include "ibeacon_model_mesh.h"
#include <esp_ble_mesh_networking_api.h>


#define CID_ESP 0x02E5

static uint8_t dev_uuid[16] = {0xdd, 0xdd};

static esp_ble_mesh_cfg_srv_t config_server = {
        .relay = ESP_BLE_MESH_RELAY_DISABLED,
        .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
        .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
        .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
        .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
        .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
        .default_ttl = 7,
        /* 3 transmissions with 20ms interval */
        .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
        .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
        ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};


// Aggiunte per stabilire la comunicazione sicura tra beacon-mesh e server-node
static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t app_key[16];
}prov_key;


// Beacon-mesh model

static esp_ble_mesh_client_op_pair_t  ibeacon_model_op_pair[] = {{ESP_BLE_MESH_IBEACON_MODEL_OP_BEACON,ESP_BLE_MESH_IBEACON_MODEL_OP_STATUS}};

static esp_ble_mesh_model_op_t ibeacon_model_op[] = {ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_IBEACON_MODEL_OP_STATUS,2),ESP_BLE_MESH_MODEL_OP_END};
static esp_ble_mesh_client_t ibeacon_model_client = {.op_pair=ibeacon_model_op_pair,.op_pair_size=ARRAY_SIZE(ibeacon_model_op_pair)};

static esp_ble_mesh_model_t custom_models[] = {
        ESP_BLE_MESH_VENDOR_MODEL(CID_ESP,ESP_BLE_MESH_IBEACON_MODEL_ID_CLIENT,ibeacon_model_op,NULL,&ibeacon_model_client)
};
static esp_ble_mesh_elem_t elements[] = {
        ESP_BLE_MESH_ELEMENT(0, root_models, custom_models)
};

int ble_mesh_init();
static void provisioning_callback(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param);
// For logging purpose only
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index);
void config_server_callback(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param);
//  Da eliminire perchè relativa alle scansioni BLE ibeacon
//void custom_ibeacon_server_callback(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param);
esp_err_t ble_beacon_mesh_send(void);
void beaconing_task(void *pvParameter);

static esp_ble_mesh_prov_t provision = {
        .uuid = dev_uuid,
#if 0   // Configure for OOB (out of bound exchange key) <Security>
.output_size = 4,
.output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
.input_size = 4,
.input_actions = ESP_BLE_MESH_PUSH,
#else
.output_actions = 0,
.output_size = 0,
#endif
        };

static esp_ble_mesh_comp_t composition = {
        .cid = CID_ESP,
        .elements = elements,
        .element_count = ARRAY_SIZE(elements)
};

void ble_mesh_get_dev_uuid();

//  Da eliminire perchè relativa all'aggiornamento di stato del server node
//  void update_ibeacon_state(uint8_t *uuid, uint16_t major, uint16_t minor, int rssi);

#endif //TEST_MESH_NETWORK_BLUETOOTH_MESH_H
