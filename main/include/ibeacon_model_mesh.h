#ifndef MESH_NETWORK_SERVER_IBEACON_MODEL_MESH_H
#define MESH_NETWORK_SERVER_IBEACON_MODEL_MESH_H

#include "esp_ble_mesh_common_api.h"

//* Model's IDs Definitions (Server and Client)
#define ESP_BLE_MESH_IBEACON_MODEL_ID_SERVER      0x1416  /*!< Custom Server Model ID */
#define ESP_BLE_MESH_IBEACON_MODEL_ID_CLIENT      0x1417  /*!< Custom Client Model ID */

//* OPCODES of the messages (same on the server)
#define ESP_BLE_MESH_IBEACON_MODEL_OP_GET           ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
#define ESP_BLE_MESH_IBEACON_MODEL_OP_BEACON        ESP_BLE_MESH_MODEL_OP_3(0x04, CID_ESP)
#define ESP_BLE_MESH_IBEACON_MODEL_OP_STATUS        ESP_BLE_MESH_MODEL_OP_3(0x05, CID_ESP)

#define ESP_BLE_MESH_GROUP_PUB_ADDR                     0xC100

typedef struct __attribute__((packed)) {
//  iBeacon fields for beacon compatibility
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int rssi;
    double distance; // d = 10^( (A - RSSI) / (10*N) )
    int counter;   // Ibeacon reading counter
} model_ibeacon_data_t;

#endif //MESH_NETWORK_SERVER_IBEACON_MODEL_MESH_H
