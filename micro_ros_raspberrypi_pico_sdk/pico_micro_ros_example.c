#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico_uart_transports.h"

#include "hardware/pwm.h"
#include "ws2812_leds/ws2812_set_rgb.h"
#include "ws2812_leds/ws2812.h"
#include <rmw_microros/rmw_microros.h>

const uint LED_PIN = 25;

void set_servo_angle(uint pin, int angle) {
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_gpio_level(pin, 500 + (angle * 1000 / 180)); // Pulse width in µs mapped to 0.5ms - 2.5ms
    pwm_set_enabled(slice, true);
}


rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_int;
std_msgs__msg__Int32 msg_int_sub;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
     if (timer != NULL) {
        printf("Timer: %d\n", msg_int.data);
        set_servo_angle(1, msg_int.data); // Contrôle servo GPIO 1
        rcl_publish(&publisher, &msg_int, NULL);
        msg_int.data = (msg_int.data + 10) % 180;
    }
}

void subscription_callback(const void * msgin);

void subscription_callback(const void * msgin)
{
    static int count = 0;
    count += 1;
    ws2812_set_rgb(2, 255, 255, 0);
    if (msgin == NULL)
        ws2812_set_rgb(2, count, 0, 25);
        return;
    ws2812_set_rgb(2, 10, 0, 0);
    const std_msgs__msg__Int32 * incoming = (const std_msgs__msg__Int32 *)msgin;
    int color = incoming->data;
    ws2812_set_rgb(2, 0, color, color);
    // const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    // int angle = atoi(msg->data.data);
    // set_servo_angle(2, angle);


    // // Envoyer un message confirmant la réception
    // msg_int.data = angle;
    // rcl_publish(&publisher, &msg_int, NULL);
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Initialize WS2812 LEDs (using PIO 1, pin 18, 800kHz)
    ws2812_init(pio1, 18, 800000.0f);
    ws2812_clear();

    // sleep_ms(1000);


    printf("Entering program...\n");
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Servo
    gpio_set_function(1, GPIO_FUNC_PWM);
    gpio_set_function(2, GPIO_FUNC_PWM);
    uint slice1 = pwm_gpio_to_slice_num(1);
    uint slice2 = pwm_gpio_to_slice_num(2);
    pwm_set_wrap(slice1, 20000); // For ~50Hz PWM (20ms period)
    pwm_set_clkdiv(slice1, 125.0); // Set clock divisor for 1us resolution
    pwm_set_wrap(slice2, 20000);
    pwm_set_clkdiv(slice2, 125.0);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    // Initialize publisher
    rcl_ret_t ret2 = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "rp2040_topic");
        

    // Initialize subscriber
    ret = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "rp2040_listener_topic"
    );

    if (ret != RCL_RET_OK) {
        ws2812_set_rgb(1, 25, 0, 0);
        return 1;
    } else {
        ws2812_set_rgb(1, 0, 20, 0);
    }

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_int_sub, &subscription_callback, ON_NEW_DATA);

    ws2812_set_rgb(0, 25, 10, 0);
    msg_int.data = 0;
    int counter = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        counter += 1;
        set_servo_angle(2, counter);
        ws2812_update(true);
    }
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&subscriber, &node);

    return 0;
}
