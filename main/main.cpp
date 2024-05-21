#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include "sensor_msgs/msg/imu.h"
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <Wire.h>
#include <SFE_HMC6343.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define IMU_CHECK(fn) { if(!(fn)){printf("Failed status on line %d. Sensor Initialization Failed! Aborting.\n",__LINE__);vTaskDelete(NULL);}}

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
SFE_HMC6343 compass;

geometry_msgs__msg__Quaternion eulerToQuaternion(float, float, float);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
 
	if (timer != NULL) {

		imu_msg.header.stamp.sec = (int32_t)(millis() / 1000);
  		imu_msg.header.stamp.nanosec = (uint32_t)(millis() % 1000) * 1000000;

		compass.readHeading(); 
  		compass.readAccel();  

	    float pitch = (compass.pitch/10.0) * (M_PI / 180.0);
        float roll = (compass.roll/10.0) * (M_PI / 180.0);
        float yaw = (compass.heading/10.0) * (M_PI / 180.0);

		imu_msg.orientation = eulerToQuaternion(roll, pitch, yaw);

		imu_msg.angular_velocity.x = 0.0;
  		imu_msg.angular_velocity.y = 0.0;
  		imu_msg.angular_velocity.z = 0.0; 

		imu_msg.linear_acceleration.x = ((float) compass.accelX / 1024.0)*9.81;   
  		imu_msg.linear_acceleration.y = ((float) compass.accelY / 1024.0)*9.81;   
  		imu_msg.linear_acceleration.z = ((float) compass.accelZ / 1024.0)*9.81; 

		printf((float) compass.accelZ/1024.0);

		RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
	}
}

void micro_ros_task(void * arg)
{
	Wire.begin();
	IMU_CHECK(compass.init());

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_imu_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"freertos_imu_publisher"));
	
	// initialize message
	imu_msg.header.frame_id.data = (char*)"imu_link";
  	imu_msg.header.frame_id.size = strlen("imu_link");
  	imu_msg.header.frame_id.capacity = strlen("imu_link");

	for (int i = 0; i < 9; ++i) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  	}

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));


	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(1000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}

geometry_msgs__msg__Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
  geometry_msgs__msg__Quaternion q;
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;

  return q;
}