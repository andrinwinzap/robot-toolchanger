#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define TOPIC_BUFFER_SIZE 64
#define UROS_RECONNECT_DELAY_MS 500
#define UROS_PING_TIMEOUT_MS 100
#define UROS_PING_ATTEMPTS 1
#define UROS_PING_INTERVAL_MS 1000
#define UROS_EXECUTOR_WAIT_MS 10

#define TOOL_CHANGER_ATTACH_SERVO_ANGLE 100
#define TOOL_CHANGER_DETACH_SERVO_ANGLE 165

#define LED_OK_PIN    GPIO_NUM_7
#define LED_ERROR_PIN GPIO_NUM_15

#define ERROR_FLAG_AGENT (1u << 0)

#define SERVO_PWM_GPIO 8
#define SERVO_MIN_PULSEWIDTH_US (500)
#define SERVO_MAX_PULSEWIDTH_US (2500)
#define SERVO_MAX_DEGREE (180)

rcl_subscription_t command_subscriber;
std_msgs__msg__Bool command_subscriber_msg;
char command_subscriber_topic[TOPIC_BUFFER_SIZE];

static const char *TAG = "tool_changer";
static size_t uart_port = UART_NUM_1;

static volatile uint32_t error_flags = 0;

static void set_error_flag(uint32_t flag)
{
    error_flags |= flag;
    gpio_set_level(LED_OK_PIN, 1);    // ok off
    gpio_set_level(LED_ERROR_PIN, 0); // error on
}

static void clear_error_flag(uint32_t flag)
{
    error_flags &= ~flag;
    if (error_flags == 0) {
        gpio_set_level(LED_ERROR_PIN, 1); // error off
        gpio_set_level(LED_OK_PIN, 0);    // ok on
    }
}

void led_init(void)
{
    gpio_num_t leds[] = {LED_OK_PIN, LED_ERROR_PIN};
    for (int i = 0; i < 2; i++) {
        gpio_set_level(leds[i], 1); // drive high before enabling output to avoid glitch
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << leds[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
}

typedef enum
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} uros_state_t;

void servo_init()
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50, // Servo frequency (50Hz)
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = SERVO_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}

uint32_t angle_to_duty(uint32_t angle)
{
    uint32_t pulsewidth = SERVO_MIN_PULSEWIDTH_US +
                          ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / SERVO_MAX_DEGREE;

    uint32_t duty = (pulsewidth * (1 << 14)) / 20000; // 20000 us = 20 ms
    return duty;
}

void servo_write_angle(uint32_t angle)
{
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void command_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    int angle = msg->data ? TOOL_CHANGER_ATTACH_SERVO_ANGLE : TOOL_CHANGER_DETACH_SERVO_ANGLE;
    ESP_LOGI(TAG, "Writing angle: %d", angle);
    servo_write_angle(angle);
}

static bool create_micro_ros_entities(
    rcl_allocator_t *allocator,
    rclc_support_t *support,
    rcl_node_t *node,
    rclc_executor_t *executor)
{
    rcl_ret_t rc;

    rc = rclc_support_init(support, 0, NULL, allocator);
    if (rc != RCL_RET_OK)
    {
        return false;
    }

    rc = rclc_node_init_default(node, "tool_changer", "", support);
    if (rc != RCL_RET_OK)
    {
        return false;
    }

    rc = rclc_subscription_init_best_effort(
        &command_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        command_subscriber_topic);
    if (rc != RCL_RET_OK)
    {
        return false;
    }

    std_msgs__msg__Bool__init(&command_subscriber_msg);

    rc = rclc_executor_init(executor, &support->context, 1, allocator);
    if (rc != RCL_RET_OK)
    {
        return false;
    }

    rc = rclc_executor_add_subscription(
        executor,
        &command_subscriber,
        &command_subscriber_msg,
        command_subscriber_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK)
    {
        return false;
    }

    return true;
}

static void destroy_micro_ros_entities(
    rclc_support_t *support,
    rcl_node_t *node,
    rclc_executor_t *executor)
{
    rclc_executor_fini(executor);
    rcl_subscription_fini(&command_subscriber, node);
    std_msgs__msg__Bool__fini(&command_subscriber_msg);
    rcl_node_fini(node);
    rclc_support_fini(support);
}

void uros_task(void *arg)
{
    (void)arg;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    uros_state_t state = WAITING_AGENT;
    int64_t last_ping_check_us = 0;

    rclc_support_t support = {0};
    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    command_subscriber = rcl_get_zero_initialized_subscription();

    for (;;)
    {
        switch (state)
        {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(UROS_PING_TIMEOUT_MS, UROS_PING_ATTEMPTS) == RMW_RET_OK)
            {
                state = AGENT_AVAILABLE;
            }
            else
            {
                set_error_flag(ERROR_FLAG_AGENT);
                vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            }
            break;

        case AGENT_AVAILABLE:
            support = (rclc_support_t){0};
            node = rcl_get_zero_initialized_node();
            executor = rclc_executor_get_zero_initialized_executor();
            command_subscriber = rcl_get_zero_initialized_subscription();

            if (create_micro_ros_entities(&allocator, &support, &node, &executor))
            {
                state = AGENT_CONNECTED;
                last_ping_check_us = esp_timer_get_time();
                clear_error_flag(ERROR_FLAG_AGENT);
                ESP_LOGI(TAG, "Agent connected");
            }
            else
            {
                ESP_LOGW(TAG, "Entity creation failed, retrying...");
                destroy_micro_ros_entities(&support, &node, &executor);
                set_error_flag(ERROR_FLAG_AGENT);
                state = WAITING_AGENT;
                vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            }
            break;

        case AGENT_CONNECTED:
            if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(UROS_EXECUTOR_WAIT_MS)) != RCL_RET_OK)
            {
                state = AGENT_DISCONNECTED;
                break;
            }

            if ((esp_timer_get_time() - last_ping_check_us) >= (UROS_PING_INTERVAL_MS * 1000LL))
            {
                if (rmw_uros_ping_agent(UROS_PING_TIMEOUT_MS, UROS_PING_ATTEMPTS) != RMW_RET_OK)
                {
                    state = AGENT_DISCONNECTED;
                    break;
                }
                last_ping_check_us = esp_timer_get_time();
            }

            vTaskDelay(pdMS_TO_TICKS(1));
            break;

        case AGENT_DISCONNECTED:
        {
            ESP_LOGW(TAG, "Agent disconnected, destroying entities...");
            set_error_flag(ERROR_FLAG_AGENT);
            rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
            if (rmw_context != NULL)
            {
                (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
            }
            destroy_micro_ros_entities(&support, &node, &executor);
            state = WAITING_AGENT;
            vTaskDelay(pdMS_TO_TICKS(UROS_RECONNECT_DELAY_MS));
            break;
        }

        default:
            state = WAITING_AGENT;
            break;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

    led_init();
    clear_error_flag(0); // all clear → ok LED on

    servo_init();

    strncpy(command_subscriber_topic, "/robot/tool_changer/attach", TOPIC_BUFFER_SIZE);
    command_subscriber_topic[TOPIC_BUFFER_SIZE - 1] = '\0';

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreatePinnedToCore(
        uros_task,
        "uros_task",
        16384,
        NULL,
        20,
        NULL,
        0);

    ESP_LOGI(TAG, "Setup Complete");
}
