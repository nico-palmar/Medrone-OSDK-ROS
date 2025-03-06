#include <ros/ros.h>
#include <dji_osdk_ros/MobileData.h>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <functional>

// Define the function signature
using CommandHandler = std::function<void(const std::vector<uint8_t>&)>;

// Define data structures for incoming requests
struct CommandAData {
    int test_int;
};

struct CommandBData {
    uint8_t flag;
    bool trigger;
};

// Function prototypes
void handleCommandA(const std::vector<uint8_t>& data);
void handleCommandB(const std::vector<uint8_t>& data);

// Command mapping
std::vector<CommandHandler> command_handlers = {
    // NOTE: PUT THE HANLDERS IN ORDER OF COMMAND IDX
    handleCommandA,
    handleCommandB
};

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData) {
    if (fromMobileData->data.empty()) {
        ROS_INFO("Received empty data from mobile");
        return;
    }

    // handle a command ID recieve (can be similar to their version)
    size_t command_id = static_cast<size_t>(fromMobileData->data[0]);
    ROS_INFO_STREAM("Received command ID: " << static_cast<int>(command_id));
    
    if (command_id >= command_handlers.size()) {
        ROS_WARN_STREAM("Unknown command ID received: " << command_id);
        return;
    } 
    // run a callback function which handles the MSDK request
    CommandHandler handler = command_handlers[command_id];
    std::vector<uint8_t> payload(fromMobileData->data.begin() + 1, fromMobileData->data.end());
    // call the hanlder with the corresponding payload
    handler(payload);
    // TODO: consider returning something from here
}

void handleCommandA(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(CommandAData)) {
        ROS_WARN("Invalid data size for Command A");
        return;
    }
    CommandAData cmdA;
    std::memcpy(&cmdA, data.data(), sizeof(CommandAData));
    ROS_INFO_STREAM("Handling Command A: param1=" << cmdA.test_int);
}

void handleCommandB(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(CommandBData)) {
        ROS_WARN("Invalid data size for Command B");
        return;
    }
    CommandBData cmdB;
    std::memcpy(&cmdB, data.data(), sizeof(CommandBData));
    ROS_INFO_STREAM("Handling Command B: flag=" << static_cast<int>(cmdB.flag) << ", bool trigger=" << cmdB.trigger);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobile_data_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber fromMobileDataSub = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, fromMobileDataSubCallback);
    
    ros::spin();
    return 0;
}


