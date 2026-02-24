#include "system/BluetoothService.h"
#include "config/AppConfig.h"
#include "esp_log.h"
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <cstring>

static const char* TAG = "BluetoothService";

// ============================================================================
// GATT Service Definition
// ============================================================================

static const ble_uuid16_t svc_uuid  = BLE_UUID16_INIT(app_cfg::BLE_PROV_SVC_UUID);
static const ble_uuid16_t ssid_uuid = BLE_UUID16_INIT(app_cfg::BLE_CHAR_SSID);
static const ble_uuid16_t pass_uuid = BLE_UUID16_INIT(app_cfg::BLE_CHAR_PASS);
static const ble_uuid16_t ws_uuid   = BLE_UUID16_INIT(app_cfg::BLE_CHAR_WS_URL);
static const ble_uuid16_t mqtt_uuid = BLE_UUID16_INIT(app_cfg::BLE_CHAR_MQTT_URL);
static const ble_uuid16_t save_uuid = BLE_UUID16_INIT(app_cfg::BLE_CHAR_SAVE);

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &ssid_uuid.u, .access_cb = BluetoothService::gattAccessHandler,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &pass_uuid.u, .access_cb = BluetoothService::gattAccessHandler,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &ws_uuid.u,   .access_cb = BluetoothService::gattAccessHandler,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &mqtt_uuid.u, .access_cb = BluetoothService::gattAccessHandler,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &save_uuid.u, .access_cb = BluetoothService::gattAccessHandler,
              .flags = BLE_GATT_CHR_F_WRITE },
            { 0 },
        },
    },
    { 0 },
};

// ============================================================================
// Singleton-like access for callbacks
// ============================================================================

static BluetoothService* s_instance = nullptr;

BluetoothService::BluetoothService() { s_instance = this; }
BluetoothService::~BluetoothService() { s_instance = nullptr; }

// ============================================================================
// Init / Deinit
// ============================================================================

esp_err_t BluetoothService::init() {
    if (initialized_) return ESP_OK;

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.sync_cb  = BluetoothService::onSync;
    ble_hs_cfg.reset_cb = BluetoothService::onReset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) { ESP_LOGE(TAG, "GATT count cfg failed: %d", rc); return ESP_FAIL; }

    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) { ESP_LOGE(TAG, "GATT add svcs failed: %d", rc); return ESP_FAIL; }

    ble_svc_gap_device_name_set(app_cfg::BLE_DEVICE_NAME);
    nimble_port_freertos_init(BluetoothService::hostTask);

    initialized_ = true;
    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

esp_err_t BluetoothService::deinit() {
    if (!initialized_) return ESP_OK;
    nimble_port_stop();
    nimble_port_deinit();
    initialized_ = false;
    connected_ = false;
    return ESP_OK;
}

// ============================================================================
// Advertising
// ============================================================================

esp_err_t BluetoothService::startAdvertising() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    beginAdvertising();
    return ESP_OK;
}

esp_err_t BluetoothService::stopAdvertising() {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    ble_gap_adv_stop();
    return ESP_OK;
}

void BluetoothService::beginAdvertising() {
    struct ble_hs_adv_fields fields = {};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                  = (uint8_t*)app_cfg::BLE_DEVICE_NAME;
    fields.name_len              = strlen(app_cfg::BLE_DEVICE_NAME);
    fields.name_is_complete      = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                      &params, BluetoothService::gapEventHandler, nullptr);

    ESP_LOGI(TAG, "Advertising as '%s'", app_cfg::BLE_DEVICE_NAME);
}

// ============================================================================
// NimBLE Host Callbacks
// ============================================================================

void BluetoothService::onSync() {
    ESP_LOGI(TAG, "BLE host synced");
    ble_hs_id_infer_auto(0, nullptr);
}

void BluetoothService::onReset(int reason) {
    ESP_LOGW(TAG, "BLE host reset, reason: %d", reason);
}

void BluetoothService::hostTask(void* param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ============================================================================
// GAP Event Handler
// ============================================================================

int BluetoothService::gapEventHandler(struct ble_gap_event* event, void* arg) {
    if (!s_instance) return 0;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Client connected");
                s_instance->connected_ = true;
                s_instance->conn_handle_ = event->connect.conn_handle;
            } else {
                s_instance->beginAdvertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Client disconnected");
            s_instance->connected_ = false;
            s_instance->beginAdvertising();
            break;

        default:
            break;
    }
    return 0;
}

// ============================================================================
// GATT Access Handler
// ============================================================================

int BluetoothService::gattAccessHandler(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (!s_instance || ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    char buf[256] = {};
    if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
    ble_hs_mbuf_to_flat(ctxt->om, buf, len, nullptr);

    const ble_uuid_t* uuid = ctxt->chr->uuid;
    auto& cfg = s_instance->pending_cfg_;

    if (ble_uuid_cmp(uuid, &ssid_uuid.u) == 0) {
        cfg.ssid.assign(buf, len);
        ESP_LOGI(TAG, "SSID: %s", cfg.ssid.c_str());
    } else if (ble_uuid_cmp(uuid, &pass_uuid.u) == 0) {
        cfg.password.assign(buf, len);
        ESP_LOGI(TAG, "Password received (len=%d)", len);
    } else if (ble_uuid_cmp(uuid, &ws_uuid.u) == 0) {
        cfg.ws_url.assign(buf, len);
        ESP_LOGI(TAG, "WS URL: %s", cfg.ws_url.c_str());
    } else if (ble_uuid_cmp(uuid, &mqtt_uuid.u) == 0) {
        cfg.mqtt_url.assign(buf, len);
        ESP_LOGI(TAG, "MQTT URL: %s", cfg.mqtt_url.c_str());
    } else if (ble_uuid_cmp(uuid, &save_uuid.u) == 0) {
        ESP_LOGI(TAG, "Save command received - triggering config complete");
        if (s_instance->config_cb_) {
            s_instance->config_cb_(cfg);
        }
    }

    return 0;
}
