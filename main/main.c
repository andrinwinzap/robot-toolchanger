#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define TOPIC_BUFFER_SIZE 64

#define TOOL_CHANGER_ATTACH_SERVO_ANGLE 10
#define TOOL_CHANGER_DETACH_SERVO_ANGLE TOOL_CHANGER_ATTACH_SERVO_ANGLE + 44

#define SERVO_PWM_GPIO 8
#define SERVO_MIN_PULSEWIDTH_US (500)
#define SERVO_MAX_PULSEWIDTH_US (2500)
#define SERVO_MAX_DEGREE (180)

#define RCCHECK(fn)                                                                           \
    {                                                                                         \
        rcl_ret_t temp_rc = fn;                                                               \
        if ((temp_rc != RCL_RET_OK))                                                          \
        {                                                                                     \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Retrying.", __LINE__, (int)temp_rc); \
            taskYIELD();                                                                      \
            goto try_uros_task;                                                               \
        }                                                                                     \
    }
#define RCSOFTCHECK(fn)                                                                         \
    {                                                                                           \
        rcl_ret_t temp_rc = fn;                                                                 \
        if ((temp_rc != RCL_RET_OK))                                                            \
        {                                                                                       \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
        }                                                                                       \
    }

rcl_subscription_t command_subscriber;
std_msgs__msg__Bool command_subscriber_msg;
char command_subscriber_topic[TOPIC_BUFFER_SIZE];

static const char *TAG = "tool_changer";
static size_t uart_port = UART_NUM_1;

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

void uros_task(void *arg)
{
try_uros_task:
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "tool_changer", "", &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        command_subscriber_topic));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &command_subscriber,
        &command_subscriber_msg,
        command_subscriber_callback,
        ON_NEW_DATA));

    rclc_executor_spin(&executor);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

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
