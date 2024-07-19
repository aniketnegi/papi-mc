#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_system.h"
#include "esp_netif.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define MOTOR 21

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC2 Channels
#define SENS_01 ADC_CHANNEL_6 // GPIO34
// #define SENS_02 ADC_CHANNEL_3 // GPIO15
// #define SENS_03 ADC_CHANNEL_4 // GPIO13
// #define SENS_04 ADC_CHANNEL_5 // GPIO12
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_UNIT ADC_UNIT_1

#define NUM_OF_SENSORS 1

int adc_raw[NUM_OF_SENSORS] = {-1};

/*---------------------------------------------------------------
        ADC Vars
---------------------------------------------------------------*/
adc_oneshot_unit_handle_t papi_adc_handle;

#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected to Wi-Fi*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "PAPI";

static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* ADC */

adc_oneshot_unit_handle_t adc_init(adc_unit_t adc_unit, adc_channel_t adc_channel, adc_atten_t adc_atten)
{

    //-------------ADC Init---------------//
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = adc_atten,
    };
    //-------------ADC Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_channel, &config));

    return adc_handle;
}

/* SENSOR TASK RUNNING IN THE BG */

SemaphoreHandle_t xMutex;

// taskHandle to control when to suspend and resume the bg sensor task
TaskHandle_t taskHandle = NULL;
TimerHandle_t xTimer = NULL;

// Sensor Thresholds
#define MOISTURE_THRESHOLD_LOW 35  // Example threshold for turning the motor on
#define MOISTURE_THRESHOLD_HIGH 80 // Example threshold for turning the motor off
#define SENSOR_BG_READ_DELAY 7000  // (val / 1000) seconds

#define SENSOR_MIN 950
#define SENSOR_MAX 2600

/* FOR DEBUG */
#define POT_MIN 0
#define POT_MAX 4096

bool motor = false;

int normalise(int val, int min, int max, int a, int b)
{
    /* NORMALISE
        take an input of `val`
        return a number between `a` and `b` such that ret is `val` scaled between `min` and `max`
    */
    return (int)(((float)(b - a) * ((float)(val - min) / (max - min))) + a);
}

int get_moisture_percentage()
{
    ESP_ERROR_CHECK(adc_oneshot_read(papi_adc_handle, SENS_01, &adc_raw[0]));

    return (100 - normalise(adc_raw[0], SENSOR_MIN, SENSOR_MAX, 0, 100));
}

static void turn_motor_on_pulse(bool motor_state)
{
    if (motor_state)
    {
        gpio_set_level(MOTOR, 1);
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        gpio_set_level(MOTOR, 0);
    }
    else
    {

        gpio_set_level(MOTOR, 0);
    }
}

// Function to control the motor based on sensor value
void control_motor_based_on_moisture(int sensor_value)
{

    if (sensor_value < MOISTURE_THRESHOLD_LOW)
    {
        ESP_LOGI(TAG, "Moisture level low (%d), turning on water", sensor_value);
        motor = true;
    }

    if (sensor_value > MOISTURE_THRESHOLD_HIGH)
    {
        ESP_LOGI(TAG, "Moisture level high (%d), turning off water", sensor_value);
        motor = false;
    }

    turn_motor_on_pulse(motor);
}

// Sensor Reading and Motor Control Task
void sensor_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
            int mp = get_moisture_percentage();

            ESP_LOGI(TAG, "Moisture level: %d", mp);
            control_motor_based_on_moisture(mp);

            xSemaphoreGive(xMutex);
        }
        vTaskDelay(SENSOR_BG_READ_DELAY / portTICK_PERIOD_MS); // Adjust the delay as needed
    }
}

// Timer callback function
void vTimerCallback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Resuming sensor_task");
    vTaskResume(taskHandle);
}

/*---------------------------------------------------------------
  ---------------------------------------------------------------
       MQTT
  ---------------------------------------------------------------
---------------------------------------------------------------*/

/*---------------------------------------------------------------
       MQTT Event Handler
---------------------------------------------------------------*/
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, "pv0/status/mc", "ONLINE", 0, 0, 0);
        // meh
        esp_mqtt_client_subscribe(client, "pv0/ping", 0);

        // subscribe to /indratest/motor/
        // should be qos2 - exactly 1ce
        msg_id = esp_mqtt_client_subscribe(client, "pv0/commands", 2);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "pv0/autodelayx", 2);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        // triggered when message is received on the subscribed topic
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (strncmp(event->topic, "pv0/commands", event->topic_len) == 0)
        {
            if (strncmp(event->data, "WATER_ON", event->data_len) == 0)
            {
                ESP_LOGI(TAG, "Turning on Water");
                gpio_set_level(MOTOR, 1);
            }
            else if (strncmp(event->data, "WATER_OFF", event->data_len) == 0)
            {
                ESP_LOGI(TAG, "Turning off Water");
                gpio_set_level(MOTOR, 0);
            }
            else if (strncmp(event->data, "MOISTURE_GET", event->data_len) == 0)
            {
                // errors unless sensor is actually connected !
                ESP_LOGI(TAG, "Checking soil moisture");
                vTaskDelay(500 / portTICK_PERIOD_MS); // Adjust the delay as needed

                // uint16_t moisture_percent = get_moisture_percentage();

                int moisture_percent = get_moisture_percentage();

                // str convert
                int length = snprintf(NULL, 0, "%d", moisture_percent);
                char *val = malloc(length + 1);
                snprintf(val, length + 1, "%d", moisture_percent);

                msg_id = esp_mqtt_client_publish(client, "pv0/moisture", val, 0, 0, 0);
                ESP_LOGI(TAG, "sent SOIL MOISTURE successful, msg_id=%d", msg_id);
                ESP_LOGI(TAG, "sent SOIL MOISTURE successful, data=%s", val);

                free(val); // thank you C !!
            }
            else if (strncmp(event->data, "MANUAL_OVERRIDE_ON", event->data_len) == 0)
            {
                ESP_LOGI(TAG, "SAYONARA: You are in control now!");
                vTaskDelay(500 / portTICK_PERIOD_MS);

                vTaskSuspend(taskHandle);
            }
            else if (strncmp(event->data, "MANUAL_OVERRIDE_OFF", event->data_len) == 0)
            {
                ESP_LOGI(TAG, "WELCOME BACK: My controls!");
                vTaskDelay(500 / portTICK_PERIOD_MS);

                vTaskResume(taskHandle);
            }
        }
        else if (strncmp(event->topic, "pv0/ping", event->topic_len) == 0)
        {
            if (strncmp(event->data, "PING", event->data_len) == 0)
            {
                msg_id = esp_mqtt_client_publish(client, "pv0/pong", "PONG", 0, 0, 0);
                ESP_LOGI(TAG, "sent PONG successful, msg_id=%d", msg_id);
            }
        }
        else if (strncmp(event->topic, "pv0/autodelayx", event->topic_len) == 0)
        {
            int delay_s;
            sscanf(event->data, "%d", &delay_s);

            ESP_LOGI(TAG, "Suspending AUTO mode for %d seconds", delay_s);

            vTaskSuspend(taskHandle);
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(delay_s * 1000), 0);
            xTimerStart(xTimer, 0);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(esp_mqtt_event_handle_t mqtt_event_handler_cb)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);
    esp_mqtt_client_start(client);
}

void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // ADC Stuff
    papi_adc_handle = adc_init(ADC_UNIT, SENS_01, ADC_ATTEN);

    // motor stuff
    gpio_reset_pin(MOTOR);
    gpio_set_direction(MOTOR, GPIO_MODE_OUTPUT);

    // Create mutex for shared resource protection
    xMutex = xSemaphoreCreateMutex();

    if (xMutex != NULL)
    {
        xTaskCreate(sensor_task, "bg sensor", 4096, NULL, tskIDLE_PRIORITY, &taskHandle);
    }

    // Create the timer for delaying the sensor task
    xTimer = xTimerCreate("SensorTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, vTimerCallback);
    if (xTimer == NULL)
    {
        ESP_LOGE(TAG, "Timer creation failed!");
    }

    // certificates problem
    // 07-07-2024@22:35 solved certificate problem by messing with AWS and docker conf. god
    // https://devopstar.com/2020/03/14/easy-aws-iot-with-esp-idf/
    mqtt_app_start(mqtt_event_handler);
}
