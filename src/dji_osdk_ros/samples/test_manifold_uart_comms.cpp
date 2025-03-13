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
    int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
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

    // Set raw mode (disable echo, input processing)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    // Save settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr: %s", strerror(errno));
        return 1;
    }

    // Flush the port
    tcflush(serial_port, TCIOFLUSH);

    // Write data
    // char msg[] = "Hello from Manifold 2-G, ON!";

    // ssize_t bytes_written = write(serial_port, msg, strlen(msg));

    // Array of 8-bit numbers to send
    uint8_t data[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

    // Send 8-bit numbers over UART
    ssize_t bytes_written = write(serial_port, data, sizeof(data));

    if (bytes_written < 0) {
        ROS_ERROR("Error writing to serial port: %s", strerror(errno));
        close(serial_port);
        return 1;
    }
    
    ROS_INFO("Sent message");
    
    close(serial_port);
    return 0;
}
