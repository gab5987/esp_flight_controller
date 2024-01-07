/*
 * REQ-TEL-000: Shall initialize both the espnow and wifi modules, providing a
 * small footprint, full-duplex comunication between the controller and the
 * device.
 *
 * REQ-TEL-001: Shall use the espnow to receive controller info and actuate into
 * the flight controller, updating the device's state machine.
 *
 * REQ-TEL-002: Shall provide a telemetry interface, sending curent data such as
 * speed, altitude, rotation, currenttime and remaining operational time.
 *
 * REQ-TEL-003: Provide data corruption and error handling into the transmited
 * data. CRCs might just enought.
 * */

#include <alloca.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdlib.h>

#include "cfg.h"

#define IS_BROADCAST_ADDR(addr) \
    (memcmp(addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN) == 0)

static const char TAG[] = "telemetry";

static const wifi_mode_t ESPNOW_WIFI_MODE = WIFI_MODE_STA;
static const byte        ESPNOW_CHANNEL   = 1;
/* Both the controller(master) and this device(slave) must share the same PMK*/
static const char ESPNOW_PMK[] = "pmkrsh05iw2aewm";
// static const size_t MAX_DATA_LEN                    = UINT_LEAST8_MAX;
// static const byte   BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff,
//                                                        0xff, 0xff, 0xff};

enum msg_type_e
{
    MSG_POSR_REQ   = 0x00,
    MSG_POS_UPDATE = 0x01,
};

struct PACKED data_exchange
{
    enum msg_type_e msg_type;
    u16             crc;
    byte            flags;
    u8              data_len;
    const byte *restrict payload;
};
typedef struct data_exchange data_exchange_t;

static void rcvcb(
    const esp_now_recv_info_t *restrict recv_info, const byte *data, i32 len)
{
    byte *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    data_exchange_t debf = {
        .msg_type = data[0],
        .crc      = (data[1] << 8) | data[2],
        .flags    = data[3],
        .data_len = data[4],
        .payload  = &data[5],
    };
}

inline static void sendcb(const u8 *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(
        TAG, "Send data to " MACSTR ", status: %d", MAC2STR(mac_addr), status);
}

esp_err_t tel_initialize(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Generating wifi configuration...");
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize wifi module");
        exit(EXIT_FAILURE);
    }

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Initializing espnow library...");
    ret = esp_now_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize espnow module");
        exit(EXIT_FAILURE);
    }

    ESP_ERROR_CHECK(esp_now_register_send_cb(sendcb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(rcvcb));
    ESP_ERROR_CHECK(esp_now_set_pmk((byte *)ESPNOW_PMK));

    return ret;
}

