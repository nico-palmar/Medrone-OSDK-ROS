#include <ros/ros.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "manifold_uart_sender");
    ros::NodeHandle nh;
    
    // Open serial port
    int serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (serial_port < 0) {
        ROS_ERROR("Error opening port: %s", strerror(errno));
        return 1;
    }

    // Configure port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
        ROS_ERROR("Error from tcgetattr: %s", strerror(errno));
        return 1;
    }

    // Set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 (8 bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Save settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr: %s", strerror(errno));
        return 1;
    }

    // Write data
    char msg[] = "Hello from Manifold 2-G!";
    write(serial_port, msg, strlen(msg));
    
    ROS_INFO("Sent message: %s", msg);
    
    close(serial_port);
    return 0;
}
