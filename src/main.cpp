#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

// imu
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>

// barometer
#include <std_msgs/msg/float32.h>

// thruster_values
#include <std_msgs/msg/u_int16_multi_array.h>

// killswitch
#include <std_msgs/msg/bool.h>

// time_stable
#include <std_msgs/msg/u_int32.h>

// setpoint
#include <geometry_msgs/msg/pose.h>

const uint8_t led_pin = 13;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 barometer_msg;
std_msgs__msg__UInt32 time_stable_msg;
std_msgs__msg__UInt16MultiArray thruster_values_msg;
geometry_msgs__msg__Pose setpoint_msg;
std_msgs__msg__Bool killswitch_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

// Error handle loop
void error_loop()
{
    while (1)
    {
        delay(100);
    }
}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    }
}

void barometer_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &barometer_msg, NULL));
    }
}

void time_stable_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &time_stable_msg, NULL));
    }
}

void thruster_values_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &thruster_values_msg, NULL));
    }
}

/**
 * @brief Setup Micro-ROS and create an imu publisher
*/
void setup_micro_ros_node()
{
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "control", "", &support));

    // create imu publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "sensors/imu"));
    
    // create barometer publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "sensors/barometer"));

    // create time_stable publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "sensors/time_stable"));

    // create thruster_values publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
        "thruster_values"));

    // create imu timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        imu_timer_callback));

    // create barometer timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        barometer_timer_callback));
    
    // create time_stable timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        time_stable_timer_callback));
    
    // create thruster_values timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        thruster_values_timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // init imu message
    imu_msg.header.frame_id.data = (char *)"imu";
    imu_msg.header.frame_id.size = 3;
    imu_msg.header.frame_id.capacity = 3;

    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nanosec = 0;

    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;

    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;

    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;

    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    // init barometer message
    barometer_msg.data = 0.0;

    // init time_stable message
    time_stable_msg.data = 0;

    // init thruster_values message
    thruster_values_msg.data.data = (uint16_t *)malloc(8 * sizeof(uint16_t));
    thruster_values_msg.data.size = 8;
    thruster_values_msg.data.capacity = 8;
    for(int i = 0; i < 8; i++)
    {
        thruster_values_msg.data.data[i] = 0;
    }
}

void setup()
{
    // Configure LED pin
    pinMode(led_pin, OUTPUT);

    // Configure micro_ros serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    setup_micro_ros_node();
}

void loop()
{
    digitalWrite(led_pin, millis() % 2000 > 1000 ? HIGH : LOW);

    // update imu message
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
