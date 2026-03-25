#include "BluetoothService.hpp"
#include "esp_log.h"
#include <cstring>
#include <algorithm>
#include <set>
#include "WifiService.hpp"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_bt.h"

static const char *TAG = "BT_SVC";

BluetoothService *BluetoothService::s_instance = nullptr;

// NimBLE characteristic value handles
static uint16_t chr_val_handles[14] = {0};

// Forward declarations for NimBLE callbacks
static int gatt_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

// Pre-declare static UUIDs (BLE_UUID16_DECLARE doesn't work in C++ static init)
static ble_uuid16_t svc_uuid  = BLE_UUID16_INIT(0xFF01);
static ble_uuid16_t chr_uuid_00 = BLE_UUID16_INIT(0xFF02);
static ble_uuid16_t chr_uuid_01 = BLE_UUID16_INIT(0xFF03);
static ble_uuid16_t chr_uuid_02 = BLE_UUID16_INIT(0xFF04);
static ble_uuid16_t chr_uuid_03 = BLE_UUID16_INIT(0xFF05);
static ble_uuid16_t chr_uuid_04 = BLE_UUID16_INIT(0xFF06);
static ble_uuid16_t chr_uuid_05 = BLE_UUID16_INIT(0xFF07);
static ble_uuid16_t chr_uuid_06 = BLE_UUID16_INIT(0xFF08);
static ble_uuid16_t chr_uuid_07 = BLE_UUID16_INIT(0xFF09);
static ble_uuid16_t chr_uuid_08 = BLE_UUID16_INIT(0xFF0A);
static ble_uuid16_t chr_uuid_09 = BLE_UUID16_INIT(0xFF0B);
static ble_uuid16_t chr_uuid_10 = BLE_UUID16_INIT(0xFF0C);
static ble_uuid16_t chr_uuid_11 = BLE_UUID16_INIT(0xFF0D);
static ble_uuid16_t chr_uuid_12 = BLE_UUID16_INIT(0xFF0E);
static ble_uuid16_t chr_uuid_13 = BLE_UUID16_INIT(0xFF0F);

#define RW_FLAGS (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE)
#define R_FLAGS  BLE_GATT_CHR_F_READ
#define W_FLAGS  BLE_GATT_CHR_F_WRITE

// GATT service definition
static const struct ble_gatt_chr_def gatt_chars[] = {
    { .uuid = &chr_uuid_00.u, .access_cb = gatt_chr_access, .arg = (void*)0,  .flags = RW_FLAGS, .val_handle = &chr_val_handles[0] },
    { .uuid = &chr_uuid_01.u, .access_cb = gatt_chr_access, .arg = (void*)1,  .flags = RW_FLAGS, .val_handle = &chr_val_handles[1] },
    { .uuid = &chr_uuid_02.u, .access_cb = gatt_chr_access, .arg = (void*)2,  .flags = RW_FLAGS, .val_handle = &chr_val_handles[2] },
    { .uuid = &chr_uuid_03.u, .access_cb = gatt_chr_access, .arg = (void*)3,  .flags = W_FLAGS,  .val_handle = &chr_val_handles[3] },
    { .uuid = &chr_uuid_04.u, .access_cb = gatt_chr_access, .arg = (void*)4,  .flags = W_FLAGS,  .val_handle = &chr_val_handles[4] },
    { .uuid = &chr_uuid_05.u, .access_cb = gatt_chr_access, .arg = (void*)5,  .flags = R_FLAGS,  .val_handle = &chr_val_handles[5] },
    { .uuid = &chr_uuid_06.u, .access_cb = gatt_chr_access, .arg = (void*)6,  .flags = R_FLAGS,  .val_handle = &chr_val_handles[6] },
    { .uuid = &chr_uuid_07.u, .access_cb = gatt_chr_access, .arg = (void*)7,  .flags = W_FLAGS,  .val_handle = &chr_val_handles[7] },
    { .uuid = &chr_uuid_08.u, .access_cb = gatt_chr_access, .arg = (void*)8,  .flags = R_FLAGS,  .val_handle = &chr_val_handles[8] },
    { .uuid = &chr_uuid_09.u, .access_cb = gatt_chr_access, .arg = (void*)9,  .flags = R_FLAGS,  .val_handle = &chr_val_handles[9] },
    { .uuid = &chr_uuid_10.u, .access_cb = gatt_chr_access, .arg = (void*)10, .flags = RW_FLAGS, .val_handle = &chr_val_handles[10] },
    { .uuid = &chr_uuid_11.u, .access_cb = gatt_chr_access, .arg = (void*)11, .flags = RW_FLAGS, .val_handle = &chr_val_handles[11] },
    { .uuid = &chr_uuid_12.u, .access_cb = gatt_chr_access, .arg = (void*)12, .flags = RW_FLAGS, .val_handle = &chr_val_handles[12] },
    { .uuid = &chr_uuid_13.u, .access_cb = gatt_chr_access, .arg = (void*)13, .flags = RW_FLAGS, .val_handle = &chr_val_handles[13] },
    { 0 } // Terminator
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = gatt_chars,
    },
    { 0 } // Terminator
};

// === NimBLE GAP event handler ===

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    auto *self = BluetoothService::s_instance;
    if (!self) return 0;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connected: conn_handle=%d, status=%d",
                 event->connect.conn_handle, event->connect.status);
        self->url_unlocked_ = false;
        if (event->connect.status != 0) {
            // Connection failed, restart advertising
            self->start();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected: reason=0x%x", event->disconnect.reason);
        self->url_unlocked_ = false;
        self->start(); // Restart advertising
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE Advertising complete");
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: conn_handle=%d, mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

// === NimBLE GATT access callback ===

static int gatt_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    auto *self = BluetoothService::s_instance;
    if (!self) return BLE_ATT_ERR_UNLIKELY;

    int idx = (int)(intptr_t)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Extract write data
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t buf[256];
        if (len > sizeof(buf)) len = sizeof(buf);
        os_mbuf_copydata(ctxt->om, 0, len, buf);

        switch (idx) {
        case 0: // DEVICE_NAME
            self->temp_cfg_.device_name.assign((char*)buf, len);
            break;
        case 1: // VOLUME
            self->temp_cfg_.volume = buf[0];
            break;
        case 2: // BRIGHTNESS
            self->temp_cfg_.brightness = buf[0];
            break;
        case 3: // WIFI_SSID
            self->temp_cfg_.ssid.assign((char*)buf, len);
            break;
        case 4: // WIFI_PASS
            self->temp_cfg_.pass.assign((char*)buf, len);
            break;
        case 7: // SAVE_CMD
            if (len > 0 && buf[0] == 0x01 && self->config_cb_)
                self->config_cb_(self->temp_cfg_);
            break;
        case 10: // WS_URL
        case 11: // MQTT_URL
        case 12: // MQTT_USER
        case 13: // MQTT_PASS
        {
            if (!self->url_unlocked_) {
                std::string token((char*)buf, len);
                if (token == BluetoothService::WS_URL_AUTH_TOKEN) {
                    self->url_unlocked_ = true;
                    ESP_LOGI(TAG, "Auth unlocked by token");
                }
            } else {
                std::string val((char*)buf, len);
                if (idx == 10) self->temp_cfg_.ws_url = val;
                else if (idx == 11) self->temp_cfg_.mqtt_url = val;
                else if (idx == 12) self->temp_cfg_.mqtt_user = val;
                else if (idx == 13) self->temp_cfg_.mqtt_pass = val;
            }
            break;
        }
        default:
            break;
        }
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        std::string response;

        switch (idx) {
        case 0: // DEVICE_NAME
            response = self->temp_cfg_.device_name;
            break;
        case 1: // VOLUME
        {
            uint8_t v = self->temp_cfg_.volume;
            os_mbuf_append(ctxt->om, &v, 1);
            return 0;
        }
        case 2: // BRIGHTNESS
        {
            uint8_t v = self->temp_cfg_.brightness;
            os_mbuf_append(ctxt->om, &v, 1);
            return 0;
        }
        case 5: // APP_VERSION
            response = app_meta::APP_VERSION;
            break;
        case 6: // BUILD_INFO
            response = std::string(app_meta::DEVICE_MODEL) + " (" + app_meta::BUILD_DATE + ")";
            break;
        case 8: // DEVICE_ID
            response = self->device_id_str_;
            break;
        case 9: // WIFI_LIST
            if (self->wifi_read_index_ < self->wifi_networks_.size()) {
                auto &net = self->wifi_networks_[self->wifi_read_index_];
                response = net.ssid + ":" + std::to_string(net.rssi);
                self->wifi_read_index_++;
            } else {
                response = "END";
                self->wifi_read_index_ = 0;
            }
            break;
        case 10: // WS_URL
            response = self->url_unlocked_ ?
                (self->temp_cfg_.ws_url.empty() ? "EMPTY" : self->temp_cfg_.ws_url) : "LOCKED";
            break;
        case 11: // MQTT_URL
            response = self->url_unlocked_ ?
                (self->temp_cfg_.mqtt_url.empty() ? "EMPTY" : self->temp_cfg_.mqtt_url) : "LOCKED";
            break;
        case 12: // MQTT_USER
            response = self->url_unlocked_ ?
                (self->temp_cfg_.mqtt_user.empty() ? "EMPTY" : self->temp_cfg_.mqtt_user) : "LOCKED";
            break;
        case 13: // MQTT_PASS
            response = self->url_unlocked_ ?
                (self->temp_cfg_.mqtt_pass.empty() ? "EMPTY" : self->temp_cfg_.mqtt_pass) : "LOCKED";
            break;
        default:
            return BLE_ATT_ERR_UNLIKELY;
        }

        os_mbuf_append(ctxt->om, response.c_str(), response.length());
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

// === NimBLE host task ===

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync callback");
    // Start advertising after sync
    if (BluetoothService::s_instance) {
        BluetoothService::s_instance->start();
    }
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: reason=%d", reason);
}

// === BluetoothService implementation ===

BluetoothService::BluetoothService()
{
    s_instance = this;
}

BluetoothService::~BluetoothService()
{
    stop();
}

bool BluetoothService::init(const std::string &adv_name, const std::vector<WifiInfo> &cached_networks, const ConfigData *current_config)
{
    static bool s_initialized = false;
    if (s_initialized) return true;

    adv_name_ = adv_name;
    if (current_config) temp_cfg_ = *current_config;

    device_id_str_ = getDeviceEfuseID();

    // Prepare WiFi list
    wifi_networks_ = cached_networks;
    std::sort(wifi_networks_.begin(), wifi_networks_.end(),
              [](const WifiInfo &a, const WifiInfo &b) { return a.rssi > b.rssi; });
    std::vector<WifiInfo> cleaned;
    std::set<std::string> seen;
    for (auto &net : wifi_networks_) {
        if (!net.ssid.empty() && seen.find(net.ssid) == seen.end()) {
            cleaned.push_back(net);
            seen.insert(net.ssid);
        }
    }
    wifi_networks_ = cleaned;
    wifi_read_index_ = 0;
    ESP_LOGI(TAG, "WiFi list: %d networks", (int)wifi_networks_.size());

    // Initialize NimBLE
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure NimBLE host
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return false;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return false;
    }

    // Set device name
    ble_svc_gap_device_name_set(adv_name_.c_str());

    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    s_initialized = true;
    ESP_LOGI(TAG, "BLE initialized (NimBLE)");
    return true;
}

bool BluetoothService::start()
{
    if (started_) return true;

    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // Connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  // General discoverable

    // Advertising data: service UUID
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    ble_uuid16_t svc_uuid = BLE_UUID16_INIT(0xFF01);
    fields.uuids16 = &svc_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return false;
    }

    // Scan response: device name
    struct ble_hs_adv_fields rsp_fields = {};
    rsp_fields.name = (uint8_t*)adv_name_.c_str();
    rsp_fields.name_len = adv_name_.length();
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return false;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return false;
    }

    started_ = true;
    ESP_LOGI(TAG, "BLE Advertising started: %s", adv_name_.c_str());
    return true;
}

void BluetoothService::stop()
{
    if (!started_) return;
    ble_gap_adv_stop();
    started_ = false;
    ESP_LOGI(TAG, "BLE Advertising stopped");
}

void BluetoothService::deinit()
{
    stop();

    // Shut down NimBLE host — this causes nimble_port_run() to return
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
        ESP_LOGI(TAG, "NimBLE port deinitialized");
    } else {
        ESP_LOGW(TAG, "nimble_port_stop failed: %d", rc);
    }

    // Release BLE controller memory back to heap
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    ESP_LOGI(TAG, "BLE controller memory released");

    s_instance = nullptr;
}
