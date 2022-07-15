#include <esp_gap_ble_api.h>
#include <scan.h>
#include "bluetooth_mesh.h"
#include "esp_ibeacon_api.h"
#include <math.h>
#include <esp_ble_mesh_local_data_operation_api.h>

#define BLUETOOTH_MESH_TAG "BLE_MESH"

esp_err_t ble_mesh_init() {

    esp_err_t err;

    ESP_LOGI("MESH_INIT", "BLE Mesh Node initialization");

    esp_ble_mesh_register_prov_callback(provisioning_callback);
    esp_ble_mesh_register_config_server_callback(config_server_callback);

    // da eliminare poichè utile solo in caso di ricezione di risposte ack
    //esp_ble_mesh_register_custom_model_callback(custom_ibeacon_server_callback);


    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(BLUETOOTH_MESH_TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_client_model_init(&custom_models[0]);
    if(err != ESP_OK){
        ESP_LOGE("MESH_INIT","Error during model client init");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV);
    if (err != ESP_OK) {
        ESP_LOGE(BLUETOOTH_MESH_TAG, "Failed to enable mesh node");
        return err;
    }

    ESP_LOGI(BLUETOOTH_MESH_TAG, "BLE Mesh Node initialization complete");
    return ESP_OK;
}
// For logging purpose only
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(BLUETOOTH_MESH_TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(BLUETOOTH_MESH_TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    prov_key.net_idx = net_idx;
    //esp_ble_mesh_model_subscribe_group_addr(esp_ble_mesh_get_primary_element_address(), CID_ESP, ESP_BLE_MESH_IBEACON_MODEL_ID_CLIENT, ESP_BLE_MESH_GROUP_PUB_ADDR);
    xTaskCreate(&beaconing_task, "beaconing_task", 2048, NULL, 5, NULL);
}

static void provisioning_callback(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {

    switch (event) {
        case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d",
                     param->prov_register_comp.err_code);
            break;
        case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d",
                     param->node_prov_enable_comp.err_code);
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                     param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                     param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
            break;
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
            prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                          param->node_prov_complete.flags, param->node_prov_complete.iv_index);
            break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
            break;
        case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
            ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d",
                     param->node_set_unprov_dev_name_comp.err_code);
            break;
        default:
            break;
    }
}

void config_server_callback(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param) {
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
            case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
                ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
                ESP_LOGI(BLUETOOTH_MESH_TAG, "net_idx 0x%04x, app_idx 0x%04x",
                         param->value.state_change.appkey_add.net_idx,
                         param->value.state_change.appkey_add.app_idx);
                ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
                memcpy(prov_key.app_key, param->value.state_change.appkey_add.app_key, 16);
                prov_key.app_idx = param->value.state_change.appkey_add.app_idx;
                esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address(),CID_ESP,ESP_BLE_MESH_IBEACON_MODEL_ID_CLIENT,prov_key.app_idx);
                break;
            case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
                ESP_LOGI(BLUETOOTH_MESH_TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
                ESP_LOGI(BLUETOOTH_MESH_TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                         param->value.state_change.mod_app_bind.element_addr,
                         param->value.state_change.mod_app_bind.app_idx,
                         param->value.state_change.mod_app_bind.company_id,
                         param->value.state_change.mod_app_bind.model_id);
                break;
            default:
                break;
        }
    }
}

void ble_mesh_get_dev_uuid() {
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
    ESP_LOG_BUFFER_HEX("dev_uuid", dev_uuid, 16);
}

void beaconing_task(void *pvParameter){
    //UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
   // ESP_LOGI("WATERMARK","Watermark: %d",uxHighWaterMark);
    while(1){
        ble_beacon_mesh_send();
        vTaskDelay(pdMS_TO_TICKS(500));
      //  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
       // ESP_LOGI("WATERMARK2","Watermark: %d",uxHighWaterMark);
    }
}

// da eliminare perchè relativa all'invio del server node inseguito alla richiesta GET del provisioner
/*

void custom_ibeacon_server_callback(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param) {
    switch (event) {
        case ESP_BLE_MESH_MODEL_OPERATION_EVT:
            switch (param->model_operation.opcode) {
                case ESP_BLE_MESH_IBEACON_MODEL_OP_GET:;
                    model_ibeacon_data_t ibeacon_resp = *(model_ibeacon_data_t *) param->model_operation.model->user_data;
                    esp_log_buffer_hex("UUID: ", ibeacon_resp.uuid, ESP_UUID_LEN_128);

                    ESP_LOGI(BLUETOOTH_MESH_TAG, "MESH MESSAGE SENT - MAJOR: %hu, MINOR: %d, RSSI: %d distance: %f - Counter #%d - \n",
                             ibeacon_resp.major, ibeacon_resp.minor, ibeacon_resp.rssi, ibeacon_resp.distance, ibeacon_resp.counter);
                    esp_err_t ib_err = esp_ble_mesh_server_model_send_msg(param->model_operation.model,
                                                                       param->model_operation.ctx,
                                                                       ESP_BLE_MESH_IBEACON_MODEL_OP_STATUS,
                                                                       sizeof(ibeacon_resp), (uint8_t *) &ibeacon_resp);
                    if (ib_err) {
                        ESP_LOGE(BLUETOOTH_MESH_TAG, "Failed to send message 0x%06x",
                                 ESP_BLE_MESH_IBEACON_MODEL_OP_STATUS);
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

*/


// Da eliminare perchè relativa allo scanner BLE iBeacon del server node
/*

static void ble_mesh_scan_cb(esp_ble_mesh_ble_cb_event_t event, esp_ble_mesh_ble_cb_param_t *param) {
    if(esp_ble_is_ibeacon_packet(param->scan_ble_adv_pkt.data, param->scan_ble_adv_pkt.length)){

        esp_ble_ibeacon_t *beacon_data= (esp_ble_ibeacon_t *) param->scan_ble_adv_pkt.data;
        ESP_LOGI("IBEACON SCAN","----- TROVATO IL BEACON ----");
        esp_log_buffer_hex("UUID: ",beacon_data->ibeacon_vendor.proximity_uuid,ESP_UUID_LEN_128);
        ESP_LOGI("IBEACON SCAN","Measured Power: %d",beacon_data->ibeacon_vendor.measured_power);

        uint16_t major = ENDIAN_CHANGE_U16(beacon_data->ibeacon_vendor.major);
        uint16_t minor = ENDIAN_CHANGE_U16(beacon_data->ibeacon_vendor.minor);

        ESP_LOGI("IBEACON SCAN","Major: %d Minor: %d",major,minor);
        ESP_LOGI("IBEACON SCAN","RSSI: %d", param->scan_ble_adv_pkt.rssi);
        ESP_LOGI("IBEACON SCAN","\n");

        update_ibeacon_state(beacon_data->ibeacon_vendor.proximity_uuid, major, minor, param->scan_ble_adv_pkt.rssi);
    }

}
*/



// da eliminare perchè relativo all'aggiornamento di stato effettuato in precedenza dal server node
/*
 void update_ibeacon_state(uint8_t *uuid, uint16_t major, uint16_t minor, int rssi) {
    // d = 10^(((P)-(S))/(10*N))
    // Where:
    //    d - estimated distance in meters
    //    P - beacon broadcast power in dBm at 1 m (Tx Power)
    //    S - measured signal value (RSSI) in dBm
    //    N - environmental factor (usually value between 2 and 4)
    int meausuredPower = -74;
    double envFactor = 5.3;

    memcpy(_ibeacon_model_state.uuid, uuid, 16);
    _ibeacon_model_state.major = major;
    _ibeacon_model_state.minor = minor;
    _ibeacon_model_state.rssi = rssi;
    _ibeacon_model_state.distance = pow(10,((double )(meausuredPower - rssi) / (10 * envFactor)));
    _ibeacon_model_state.counter++;
}
 */

esp_err_t ble_beacon_mesh_send(void){
    esp_ble_mesh_msg_ctx_t ctx = {0};
    uint32_t opcode;
    esp_err_t err = ESP_OK;
    opcode = ESP_BLE_MESH_IBEACON_MODEL_OP_BEACON;

        ctx.net_idx = prov_key.net_idx;
        ctx.app_idx = prov_key.app_idx;
        ctx.addr = ESP_BLE_MESH_GROUP_PUB_ADDR;
        ctx.send_ttl = 7;
        ctx.send_rel = false;

        /*model_ibeacon_data_t* ibeacon_resp = (model_ibeacon_data_t *) ibeacon_model_client.model->user_data;

        memcpy(ibeacon_resp->uuid, dev_uuid, 16);
        ibeacon_resp->minor = 90;
        ibeacon_resp->major = 69;
        ibeacon_resp->counter = 0;
        ibeacon_resp->distance = 0.0;
        ibeacon_resp->rssi = 0;*/


        err = esp_ble_mesh_client_model_send_msg(ibeacon_model_client.model, &ctx, opcode, 0, NULL, 0, false, ROLE_NODE);
        if (err != ESP_OK)
            ESP_LOGE("SEND_GET", "Sending error\n");
        else
            ESP_LOGI("SEND","Beaconing message sent");

    return err;
}