#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"

// --------------------------------------------------------------------------------------
// Configuration
// --------------------------------------------------------------------------------------

static const char *TAG = "SPI_BRIDGE";

static constexpr const char *WIFI_SSID = "YOUR_SSID";
static constexpr const char *WIFI_PASS = "YOUR_PASSWORD";
static constexpr const char *HOSTNAME  = "esp32c6-spi-bridge";
static constexpr uint16_t TCP_PORT = 3333;

// SPI pins (adjust to your wiring)
static constexpr gpio_num_t PIN_MOSI = GPIO_NUM_6;
static constexpr gpio_num_t PIN_MISO = GPIO_NUM_5;
static constexpr gpio_num_t PIN_SCLK = GPIO_NUM_7;
static constexpr gpio_num_t PIN_SS   = GPIO_NUM_4;

static constexpr size_t FRAME_DATA_LEN = 512;
static constexpr size_t FRAME_LEN = 2 + FRAME_DATA_LEN; // 514 bytes
static constexpr size_t MAX_LINE_LEN = FRAME_DATA_LEN;   // includes trailing \n

static constexpr size_t TCP_RX_QUEUE_DEPTH = 4;
static constexpr size_t SPI_RX_QUEUE_DEPTH = 4;

// --------------------------------------------------------------------------------------
// Data structures
// --------------------------------------------------------------------------------------

struct LineBuffer {
    size_t len;
    uint8_t data[MAX_LINE_LEN];
};

static EventGroupHandle_t wifi_event_group;
static QueueHandle_t tcp_to_spi_queue;
static QueueHandle_t spi_to_tcp_queue;

static constexpr int WIFI_CONNECTED_BIT = BIT0;

// --------------------------------------------------------------------------------------
// Utilities
// --------------------------------------------------------------------------------------

static inline void frame_clear(uint8_t *buf) {
    memset(buf, 0, FRAME_LEN);
}

static inline void frame_fill(uint8_t *buf, const uint8_t *payload, size_t len) {
    size_t capped = len > FRAME_DATA_LEN ? FRAME_DATA_LEN : len;
    buf[0] = static_cast<uint8_t>(capped & 0xFF);
    buf[1] = static_cast<uint8_t>((capped >> 8) & 0xFF);
    if (capped) {
        memcpy(buf + 2, payload, capped);
        if (FRAME_DATA_LEN > capped) {
            memset(buf + 2 + capped, 0, FRAME_DATA_LEN - capped);
        }
    } else {
        memset(buf + 2, 0, FRAME_DATA_LEN);
    }
}

static inline size_t frame_extract(const uint8_t *buf, uint8_t *out) {
    size_t len = static_cast<size_t>(buf[0]) | (static_cast<size_t>(buf[1]) << 8);
    if (len > FRAME_DATA_LEN) {
        len = FRAME_DATA_LEN;
    }
    if (len) {
        memcpy(out, buf + 2, len);
    }
    return len;
}

// --------------------------------------------------------------------------------------
// Wi-Fi STA setup
// --------------------------------------------------------------------------------------

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "Wi-Fi connected, IP: %s", ip4addr_ntoa(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, HOSTNAME));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID, sizeof(wifi_config.sta.ssid));
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done, waiting for connection...");
}

// --------------------------------------------------------------------------------------
// TCP server task
// --------------------------------------------------------------------------------------

static int create_listen_socket(void) {
    struct sockaddr_in6 listen_addr = {};
    listen_addr.sin6_family = AF_INET6;
    listen_addr.sin6_addr = in6addr_any;
    listen_addr.sin6_port = htons(TCP_PORT);

    int listen_sock = socket(AF_INET6, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, reinterpret_cast<struct sockaddr *>(&listen_addr), sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        return -1;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP server listening on port %u", TCP_PORT);
    return listen_sock;
}

static void handle_client(int client_sock) {
    ESP_LOGI(TAG, "Client connected");
    fcntl(client_sock, F_SETFL, O_NONBLOCK);

    char rx_line[MAX_LINE_LEN];
    size_t rx_len = 0;

    while (true) {
        // Handle outbound data (SPI -> TCP)
        LineBuffer outgoing;
        while (xQueueReceive(spi_to_tcp_queue, &outgoing, 0) == pdTRUE) {
            ssize_t sent = send(client_sock, outgoing.data, outgoing.len, 0);
            if (sent < 0) {
                ESP_LOGW(TAG, "Send failed: errno %d", errno);
                goto disconnect;
            }
        }

        // Handle inbound data (TCP -> SPI)
        char ch;
        ssize_t received = recv(client_sock, &ch, 1, 0);
        if (received < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            ESP_LOGW(TAG, "Recv failed: errno %d", errno);
            break;
        } else if (received == 0) {
            ESP_LOGI(TAG, "Client disconnected");
            break;
        }

        if (ch == '\r') {
            continue;
        }

        if (rx_len < MAX_LINE_LEN) {
            rx_line[rx_len++] = ch;
        }

        if (ch == '\n') {
            LineBuffer line = {};
            line.len = rx_len > MAX_LINE_LEN ? MAX_LINE_LEN : rx_len;
            memcpy(line.data, rx_line, line.len);
            if (xQueueSend(tcp_to_spi_queue, &line, pdMS_TO_TICKS(10)) != pdTRUE) {
                ESP_LOGW(TAG, "tcp_to_spi_queue full, dropping line");
            }
            rx_len = 0;
        } else if (rx_len >= MAX_LINE_LEN) {
            ESP_LOGW(TAG, "Line exceeded %zu bytes, resetting buffer", MAX_LINE_LEN);
            rx_len = 0;
        }
    }

disconnect:
    shutdown(client_sock, SHUT_RDWR);
    close(client_sock);
}

static void tcp_server_task(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & WIFI_CONNECTED_BIT)) {
            continue;
        }

        int listen_sock = create_listen_socket();
        if (listen_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        struct sockaddr_in6 source_addr = {};
        socklen_t addr_len = sizeof(source_addr);
        while (true) {
            int client_sock = accept(listen_sock, reinterpret_cast<struct sockaddr *>(&source_addr), &addr_len);
            if (client_sock < 0) {
                ESP_LOGW(TAG, "accept failed: errno %d", errno);
                break;
            }

            handle_client(client_sock);
        }

        close(listen_sock);
        ESP_LOGI(TAG, "Restarting TCP server after error");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --------------------------------------------------------------------------------------
// SPI slave task
// --------------------------------------------------------------------------------------

static void spi_slave_task(void *pvParameters) {
    (void)pvParameters;

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = static_cast<int>(FRAME_LEN)
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = PIN_SS,
        .flags = 0,
        .queue_size = 1,
        .mode = 0,
        .post_setup_cb = nullptr,
        .post_trans_cb = nullptr
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI slave initialized");

    uint8_t tx_frame[FRAME_LEN];
    uint8_t rx_frame[FRAME_LEN];
    uint8_t rx_payload[FRAME_DATA_LEN];

    while (true) {
        LineBuffer line = {};
        if (xQueueReceive(tcp_to_spi_queue, &line, 0) == pdTRUE) {
            frame_fill(tx_frame, line.data, line.len);
        } else {
            frame_clear(tx_frame);
        }

        spi_slave_transaction_t t = {};
        t.length = FRAME_LEN * 8;
        t.tx_buffer = tx_frame;
        t.rx_buffer = rx_frame;

        esp_err_t err = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        size_t rx_len = frame_extract(rx_frame, rx_payload);
        if (rx_len > 0) {
            LineBuffer outgoing = {};
            outgoing.len = rx_len;
            memcpy(outgoing.data, rx_payload, rx_len);
            if (xQueueSend(spi_to_tcp_queue, &outgoing, pdMS_TO_TICKS(10)) != pdTRUE) {
                ESP_LOGW(TAG, "spi_to_tcp_queue full, dropping frame");
            }
        }
    }
}

// --------------------------------------------------------------------------------------
// Application entry point
// --------------------------------------------------------------------------------------

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C6 SPI slave bridge starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    tcp_to_spi_queue = xQueueCreate(TCP_RX_QUEUE_DEPTH, sizeof(LineBuffer));
    spi_to_tcp_queue = xQueueCreate(SPI_RX_QUEUE_DEPTH, sizeof(LineBuffer));
    configASSERT(tcp_to_spi_queue != nullptr && spi_to_tcp_queue != nullptr);

    xTaskCreatePinnedToCore(spi_slave_task, "spi_slave_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server_task", 4096, nullptr, 4, nullptr, 1);
}


