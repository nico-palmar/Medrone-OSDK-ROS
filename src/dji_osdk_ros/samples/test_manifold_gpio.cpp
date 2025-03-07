#include <ros/ros.h>
#include <dji_osdk_ros/MFIO.h>

namespace osdk = dji_osdk_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpio_control_client");
    ros::NodeHandle nh;

    // Create a service client for the 'mfio_control' service
    ros::ServiceClient client = nh.serviceClient<osdk::MFIO>("/mfio_control");

    // Prepare the service request and response
    osdk::MFIO io_service;

    // Set parameters for GPIO control
    // remove the line below if it works; its for PWM
    // io_service.request.action = osdk::MFIO::Request::TURN_ON;  // Turn the GPIO pin ON
    io_service.request.mode = osdk::MFIO::Request::MODE_GPIO_OUT;  // GPIO output mode
    // TODO: ensure the channel is configured properly in the dji assistant 2
    io_service.request.channel = 0;  // Specify which GPIO channel (0-7)
    io_service.request.gpio_value = 1;  // 1 for HIGH (ON), 0 for LOW (OFF)
    io_service.request.block = false;  // Non-blocking call (can be set to true if needed)

    // Call the service
    if (client.call(io_service))
    {
        ROS_INFO_STREAM("GPIO Pin " << io_service.request.channel << " turned ON.");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service mfio_control");
    }

    // keep the gpio pin on for 500 ms
    ros::Duration(0.5).sleep();

    // To turn OFF the GPIO pin
    io_service.request.gpio_value = 0; 

    if (client.call(io_service))
    {
        ROS_INFO_STREAM("GPIO Pin " << io_service.request.channel << " turned OFF.");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service mfio_control");
    }

    return 0;
}
