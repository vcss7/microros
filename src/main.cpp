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

// bno055
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// bar30
#include <MS5837.h>

const uint8_t led_pin = 13;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
MS5837 bar30;

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t imu_publisher;
rcl_publisher_t barometer_publisher;
rcl_publisher_t time_stable_publisher;
rcl_publisher_t thruster_values_publisher;

rcl_subscription_t killswitch_subscriber;
rcl_subscription_t setpoints_subscriber;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 barometer_msg;
std_msgs__msg__UInt32 time_stable_msg;
std_msgs__msg__UInt16MultiArray thruster_values_msg;
geometry_msgs__msg__Pose setpoints_msg;
std_msgs__msg__Bool killswitch_msg;

rclc_executor_t executor;
rclc_executor_t killswitch_executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

// killswitch callback
void killswitch_subscription_callback(const void *msgin)
{
    // Cast received message to used type
    const std_msgs__msg__Bool *killswitch_msg = (const std_msgs__msg__Bool *)msgin;

    // Print received message

    digitalWrite(led_pin, killswitch_msg->data);

    delay(3000);
}

// setpoint callback
void setpoints_subscription_callback(const void *msgin)
{
    // Cast received message to used type
    const geometry_msgs__msg__Pose *setpoint_msg = (const geometry_msgs__msg__Pose *)msgin;

    digitalWrite(led_pin, HIGH);

    delay(3000);
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
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "sensors/imu"));

    // create barometer publisher
    RCCHECK(rclc_publisher_init_default(
        &barometer_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "sensors/barometer"));

    // create time_stamp publisher
    RCCHECK(rclc_publisher_init_default(
        &time_stable_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "sensors/time_stable"));

    // create killswitch subscriber
    RCCHECK(rclc_subscription_init_default(
        &killswitch_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "actuators/killswitch"));

    // create setpoint subscriber
    RCCHECK(rclc_subscription_init_default(
        &setpoints_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "controller/setpoints"));

    // create executor
    RCCHECK(rclc_executor_init(&killswitch_executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_subscription(&killswitch_executor, &killswitch_subscriber, &killswitch_msg, &killswitch_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &setpoints_subscriber, &setpoints_msg, &setpoints_subscription_callback, ON_NEW_DATA));
}

void setup_imu()
{
    // Configure BNO055
    while (!bno.begin())
    {
        // Serial.print("BNO055 not detected\n");
        delay(1000);
    }

    bno.setExtCrystalUse(true);
}

void setup_barometer()
{
    // Configure Bar30
    while (!bar30.init())
    {
        // Serial.print("Bar30 not detected\n");
        delay(1000);
    }
    bar30.setModel(MS5837::MS5837_30BA);
    bar30.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void setup()
{
    Serial.begin(115200);

    // Configure LED pin
    pinMode(led_pin, OUTPUT);

    // setup imu and barometer
    setup_imu();
    // setup_barometer();

    // Configure micro_ros serial transport
    set_microros_serial_transports(Serial);
    delay(2000);

    setup_micro_ros_node();

    // turn on led
    digitalWrite(led_pin, HIGH);
    delay(3000);
}

void loop()
{
    digitalWrite(led_pin, millis() % 2000 > 1000);

    // update imu message
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();
    imu_msg.orientation.w = quat.w();

    imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = lin_accel.x();
    imu_msg.linear_acceleration.y = lin_accel.y();
    imu_msg.linear_acceleration.z = lin_accel.z();

    imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = ang_vel.x();
    imu_msg.angular_velocity.y = ang_vel.y();
    imu_msg.angular_velocity.z = ang_vel.z();

    // update barometer message
    barometer_msg.data += 0.1;

    // update time_stable message
    time_stable_msg.data = millis();

    // publish messages
    rcl_ret_t imu_rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
    rcl_ret_t bar_rc = rcl_publish(&barometer_publisher, &barometer_msg, NULL);
    rcl_ret_t time_rc = rcl_publish(&time_stable_publisher, &time_stable_msg, NULL);

    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&killswitch_executor, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
