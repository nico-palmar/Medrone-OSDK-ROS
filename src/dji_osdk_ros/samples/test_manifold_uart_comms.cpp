#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <string>
#include <cstring>

namespace impulse_comms

{
class UARTPublisher {
public:
    UARTPublisher(const std::string& uart_device = "/dev/ttyUSB0")
        : uart_device_(uart_device) {

        // Initialize password buffer
        str_password = std::to_string(PASSWORD) + "\n";
        strcpy(BUFFER_PASSWORD, str_password.c_str());

        // Set up subscriber
        drop_trigger_sub_ = nh_.subscribe("drop_trigger", 10,
                                          &UARTPublisher::dropTriggerCallback, this);

        ROS_INFO("UART publisher initialized. Listening for trigger commands...");
    }
    
    ~UARTPublisher() {
        // Cleanup if needed
    }

    // Function to configure and open UART
    int openUART() {
        int serial_port = open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port < 0) {
            ROS_ERROR("Error opening port: %s", strerror(errno));
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_port, &tty) != 0) {
            ROS_ERROR("Error from tcgetattr: %s", strerror(errno));
            close(serial_port);
            return -1;
        }

        // Set baud rate
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // 8N1 (8 bits, no parity, 1 stop bit)
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Set raw mode (disable echo, input processing)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        // Save settings
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            ROS_ERROR("Error from tcsetattr: %s", strerror(errno));
            close(serial_port);
            return -1;
        }

        tcflush(serial_port, TCIOFLUSH);
        return serial_port;
    }

private:
    // Callback function for the subscriber
    void dropTriggerCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        if (msg->data == PASSWORD) {
            ROS_INFO("Correct password received, sending UART command...");

            const auto serial_port = openUART();
            if (serial_port < 0) {
                ROS_ERROR("Failed to open UART");
                return;
            }

            ssize_t bytes_written = write(serial_port, BUFFER_PASSWORD, strlen(BUFFER_PASSWORD));

            if (bytes_written < 0) {
                ROS_ERROR("Error writing to serial port: %s", strerror(errno));
            } else {
                ROS_INFO("Message sent over UART");
            }

            close(serial_port);
        } else {
            ROS_WARN("Incorrect password received: %u", msg->data);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber drop_trigger_sub_;
    
    const uint32_t PASSWORD = 18922601;
    std::string str_password;
    char BUFFER_PASSWORD[10];
    
    std::string uart_device_;
};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "manifold_uart_publisher");
    // Create an instance of the class
    impulse_comms::UARTPublisher uart_publisher;
    ros::spin();
    return 0;
}
