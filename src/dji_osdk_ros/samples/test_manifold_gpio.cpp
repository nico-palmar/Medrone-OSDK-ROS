#include <ros/ros.h>
#include <dji_osdk_ros/MFIO.h>
#include <dji_osdk_ros/MFIOConfig.h>

namespace osdk = dji_osdk_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpio_control_client");
    ros::NodeHandle nh;

    // Create a service client for the 'mfio_control' service
    // ros::ServiceClient io_control_client = nh.serviceClient<osdk::MFIO>("mfio_control");

    // // Prepare the service request and response
    // osdk::MFIO io_service;

    // // Set parameters for GPIO control
    // // remove the line below if it works; its for PWM
    // io_service.request.action = osdk::MFIO::Request::TURN_ON;  // Turn the GPIO pin ON
    // io_service.request.mode = osdk::MFIO::Request::MODE_GPIO_OUT;  // GPIO output mode
    // // TODO: ensure the channel is configured properly in the dji assistant 2
    // io_service.request.channel = 1;  // Specify which GPIO channel (0-7)
    // io_service.request.gpio_value = 1;  // 1 for HIGH (ON), 0 for LOW (OFF)
    // io_service.request.block = true;  // Non-blocking call (can be set to true if needed)

    // // Call the service
    // if (io_control_client.call(io_service))
    // {
    //     const auto value = io_service.response.read_value;
    //     std::string msg = "GPIO Pin turned ON";
    //     ROS_INFO("%s", msg.c_str());
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Failed to call service mfio_control");
    // }

    // // keep the gpio pin on for 500 ms
    // ros::Duration(15).sleep();

    // // To turn OFF the GPIO pin
    // io_service.request.action = osdk::MFIO::Request::TURN_OFF;
    // io_service.request.gpio_value = 0; 

    // if (io_control_client.call(io_service))
    // {
    //     ROS_INFO_STREAM("GPIO Pin turned OFF.");
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Failed to call service mfio_control");
    // }

    // Ensure ROS Service Client is properly initialized
    ros::ServiceClient io_control_client = nh.serviceClient<osdk::MFIO>("mfio_control");

    osdk::MFIO io_service;
    io_service.request.action = osdk::MFIO::Request::TURN_ON;  // Turn the GPIO pin ON
    io_service.request.channel = osdk::MFIO::Request::CHANNEL_1;  // Set your correct GPIO channel
    io_service.request.mode = osdk::MFIO::Request::MODE_GPIO_OUT;  // GPIO Output Mode
    io_service.request.block = true;

    // Set LOW first (optional test step)
    io_service.request.gpio_value = 0;
    if (io_control_client.call(io_service))
    {
        ROS_INFO("GPIO set to LOW (0V)");
        ros::Duration(1.0).sleep();  // Small delay
    }
    else
    {
        ROS_ERROR("Failed to set GPIO LOW");
    }

    // Now, set it to HIGH
    io_service.request.gpio_value = 1;
    if (io_control_client.call(io_service))
    {
        ROS_INFO("GPIO set to HIGH (3.3V)");
    }
    else
    {
        ROS_ERROR("Failed to set GPIO HIGH");
    }


    return 0;
}
