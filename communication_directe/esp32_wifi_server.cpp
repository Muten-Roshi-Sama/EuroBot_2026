/*
 * ESP32-S WiFi Server avec contrôle LED via JSON (ESP-IDF)
 * 
 * Ce code crée un serveur WiFi TCP qui reçoit des commandes JSON
 * pour contrôler une LED et renvoie des réponses JSON.
 * Compatible avec l'extension ESP-IDF pour VS Code
 */

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "cJSON.h"

static const char *TAG = "WIFI_SERVER";

// ========== CONFIGURATION ==========
// Modifiez ces valeurs selon votre réseau WiFi
static constexpr const char *WIFI_SSID = "VOTRE_SSID";
static constexpr const char *WIFI_PASS = "VOTRE_MOT_DE_PASSE";
static constexpr uint16_t TCP_PORT = 3333;

// Pin de la LED (LED intégrée sur la plupart des ESP32-S)
// Pour ESP32-S2: GPIO 2
// Pour ESP32-S3: GPIO 2 ou GPIO 48
// Ajustez selon votre carte
static constexpr gpio_num_t LED_PIN = GPIO_NUM_2;

// ========== VARIABLES GLOBALES ==========
static EventGroupHandle_t wifi_event_group;
static constexpr int WIFI_CONNECTED_BIT = BIT0;
static bool ledState = false;

// ========== FONCTIONS ==========

// Gestionnaire d'événements Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base, 
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi déconnecté, reconnexion...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = static_cast<ip_event_got_ip_t *>(event_data);
        ESP_LOGI(TAG, "Wi-Fi connecté! IP: %s", ip4addr_ntoa(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialisation Wi-Fi en mode Station
static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                                        &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                                        &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID, sizeof(wifi_config.sta.ssid));
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Initialisation Wi-Fi terminée, attente de connexion...");
}

// Création du socket d'écoute
static int create_listen_socket(void) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Impossible de créer le socket: errno %d", errno);
        return -1;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in listen_addr = {};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_port = htons(TCP_PORT);

    if (bind(listen_sock, reinterpret_cast<struct sockaddr *>(&listen_addr), sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG, "Erreur bind: errno %d", errno);
        close(listen_sock);
        return -1;
    }

    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Erreur listen: errno %d", errno);
        close(listen_sock);
        return -1;
    }

    ESP_LOGI(TAG, "Serveur TCP en écoute sur le port %u", TCP_PORT);
    return listen_sock;
}

// Traiter une commande JSON et renvoyer la réponse
static void process_command(const char *json_str, int client_sock) {
    cJSON *json = cJSON_Parse(json_str);
    if (json == nullptr) {
        ESP_LOGE(TAG, "Erreur de parsing JSON");
        const char *error_response = "{\"status\":\"error\",\"message\":\"Format JSON invalide\"}\n";
        send(client_sock, error_response, strlen(error_response), 0);
        return;
    }

    cJSON *command_item = cJSON_GetObjectItem(json, "command");
    if (command_item == nullptr || !cJSON_IsString(command_item)) {
        ESP_LOGE(TAG, "Champ 'command' manquant ou invalide");
        const char *error_response = "{\"status\":\"error\",\"message\":\"Champ 'command' manquant\"}\n";
        send(client_sock, error_response, strlen(error_response), 0);
        cJSON_Delete(json);
        return;
    }

    const char *command = command_item->valuestring;
    cJSON *response = cJSON_CreateObject();

    if (strcmp(command, "led_on") == 0) {
        // Allumer la LED
        gpio_set_level(LED_PIN, 1);
        ledState = true;
        ESP_LOGI(TAG, "LED allumée");

        cJSON_AddStringToObject(response, "status", "success");
        cJSON_AddStringToObject(response, "message", "led allumée");
        cJSON_AddBoolToObject(response, "led_state", true);

    } else if (strcmp(command, "led_off") == 0) {
        // Éteindre la LED
        gpio_set_level(LED_PIN, 0);
        ledState = false;
        ESP_LOGI(TAG, "LED éteinte");

        cJSON_AddStringToObject(response, "status", "success");
        cJSON_AddStringToObject(response, "message", "led éteinte");
        cJSON_AddBoolToObject(response, "led_state", false);

    } else if (strcmp(command, "led_status") == 0) {
        // Demander l'état de la LED
        cJSON_AddStringToObject(response, "status", "success");
        cJSON_AddBoolToObject(response, "led_state", ledState);
        cJSON_AddStringToObject(response, "message", ledState ? "led allumée" : "led éteinte");

    } else {
        // Commande inconnue
        ESP_LOGW(TAG, "Commande inconnue: %s", command);
        cJSON_AddStringToObject(response, "status", "error");
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg), "Commande inconnue: %s", command);
        cJSON_AddStringToObject(response, "message", error_msg);
    }

    // Convertir la réponse en JSON string et l'envoyer
    char *response_str = cJSON_Print(response);
    if (response_str) {
        char response_with_newline[512];
        snprintf(response_with_newline, sizeof(response_with_newline), "%s\n", response_str);
        send(client_sock, response_with_newline, strlen(response_with_newline), 0);
        ESP_LOGI(TAG, "Réponse envoyée: %s", response_str);
        free(response_str);
    }

    cJSON_Delete(response);
    cJSON_Delete(json);
}

// Gestion d'un client connecté
static void handle_client(int client_sock) {
    ESP_LOGI(TAG, "Client connecté!");

    char rx_buffer[512];
    char line_buffer[512];
    size_t line_len = 0;

    while (true) {
        ssize_t received = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (received < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            ESP_LOGW(TAG, "Erreur recv: errno %d", errno);
            break;
        } else if (received == 0) {
            ESP_LOGI(TAG, "Client déconnecté");
            break;
        }

        rx_buffer[received] = '\0';

        // Traiter caractère par caractère pour construire les lignes
        for (int i = 0; i < received; i++) {
            char c = rx_buffer[i];
            if (c == '\r') {
                continue; // Ignorer \r
            }
            if (c == '\n') {
                // Ligne complète reçue
                line_buffer[line_len] = '\0';
                if (line_len > 0) {
                    ESP_LOGI(TAG, "Message reçu: %s", line_buffer);
                    process_command(line_buffer, client_sock);
                }
                line_len = 0;
            } else if (line_len < sizeof(line_buffer) - 1) {
                line_buffer[line_len++] = c;
            } else {
                // Buffer plein, réinitialiser
                ESP_LOGW(TAG, "Ligne trop longue, réinitialisation");
                line_len = 0;
            }
        }
    }

    shutdown(client_sock, SHUT_RDWR);
    close(client_sock);
    ESP_LOGI(TAG, "Connexion fermée");
}

// Tâche serveur TCP
static void tcp_server_task(void *pvParameters) {
    (void)pvParameters;

    while (true) {
        // Attendre la connexion Wi-Fi
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 
                                               pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & WIFI_CONNECTED_BIT)) {
            continue;
        }

        int listen_sock = create_listen_socket();
        if (listen_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        struct sockaddr_in source_addr = {};
        socklen_t addr_len = sizeof(source_addr);
        
        while (true) {
            int client_sock = accept(listen_sock, reinterpret_cast<struct sockaddr *>(&source_addr), &addr_len);
            if (client_sock < 0) {
                ESP_LOGW(TAG, "accept échoué: errno %d", errno);
                break;
            }

            fcntl(client_sock, F_SETFL, O_NONBLOCK);
            handle_client(client_sock);
        }

        close(listen_sock);
        ESP_LOGI(TAG, "Redémarrage du serveur TCP après erreur");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Point d'entrée principal
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S WiFi Server - Démarrage");

    // Initialisation NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configuration GPIO pour la LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    ledState = false;
    ESP_LOGI(TAG, "LED configurée sur GPIO %d", LED_PIN);

    // Initialisation Wi-Fi
    wifi_init_sta();

    // Création de la tâche serveur TCP
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4096, nullptr, 5, nullptr, 1);

    ESP_LOGI(TAG, "Application démarrée, attente de connexion Wi-Fi...");
}

