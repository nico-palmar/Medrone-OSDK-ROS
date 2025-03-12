#include <ros/ros.h>
#include <dji_osdk_ros/MobileData.h>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <actionlib/client/simple_action_client.h>
#include <dji_osdk_ros/MissionAction.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <dji_osdk_ros/common_type.h>

// Define the function signature

namespace osdk = dji_osdk_ros;

using CommandHandler = std::function<void(const std::vector<uint8_t>&)>;

uint32_t PASSWORD = 46000636;
uint8_t DROP_FLAG = 42;

// Define data structures for incoming requests
struct CommandAData {
    double test_val;
};

struct CommandBData {
    uint8_t flag;
    bool trigger;
};

struct TriggerDropData {
    uint32_t password;
    uint8_t drop_flag;
};

struct RelativeMissionData
{
    uint32_t password;
    double north;
    double east;
    double up;
    double max_distance;
};

struct AbsoluteMissionData
{
    uint32_t password;
    double latitude;
    double longitude;
    double altitude;
    double max_distance;
};

// Function prototypes
void handleCommandA(const std::vector<uint8_t>& data);
void handleCommandB(const std::vector<uint8_t>& data);

// Command mapping
std::vector<CommandHandler> command_handlers = {
    // NOTE: PUT THE HANLDERS IN ORDER OF COMMAND IDX
    handleCommandA,
    handleCommandB,
    handleDropTrigger,
    handleAbsoluteMission,
    handleRelativeMission
};

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData) {
    if (fromMobileData->data.empty())
    {
        ROS_INFO("Received empty data from mobile");
        return;
    }

    // handle a command ID recieve (can be similar to their version)
    uint8_t command_id = static_cast<uint8_t>(fromMobileData->data[0]);
    ROS_INFO_STREAM("Received command ID: " << static_cast<int>(command_id));
    
    if (command_id >= command_handlers.size())
    {
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
    if (data.size() < sizeof(CommandAData))
    {
        ROS_WARN("Invalid data size for Command A");
        return;
    }
    CommandAData cmdA;
    std::memcpy(&cmdA, data.data(), sizeof(CommandAData));
    ROS_INFO_STREAM("Handling Command A: param1=" << std::fixed << std::setprecision(6) << static_cast<double>(cmdA.test_val));
}

void handleCommandB(const std::vector<uint8_t>& data){
    if (data.size() < sizeof(CommandBData))
    {
        ROS_WARN("Invalid data size for Command B");
        return;
    }
    CommandBData cmdB;
    std::memcpy(&cmdB, data.data(), sizeof(CommandBData));
    ROS_INFO_STREAM("Handling Command B: flag=" << static_cast<int>(cmdB.flag) << ", bool trigger=" << cmdB.trigger);
}

void handleDropTrigger(const std::vector<uint8_t>& data){
    if (data.size() < sizeof(TriggerDropData))
    {
        ROS_WARN("Invalid data size for Drop Trigger Command");
        return;
    }
    TriggerDropData trigger_drop_cmd;
    std::memcpy(&trigger_drop_cmd, data.data(), sizeof(TriggerDropData));
    
    // check the password fields for drop triggering
    if (!(trigger_drop_cmd.password == PASSWORD && trigger_drop_cmd.drop_flag == DROP_FLAG))
    {
        ROS_WARN("Invalid drop combination provided, rejecting drop request");
        return;
    }
    ROS_INFO("Drop command recieved; triggering");
    // TODO: Trigger a drop from here
    // likely will send a signal to the UART node via topic, which then sends drop over uart
}

void handleAbsoluteMission(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(AbsoluteMissionData))
    {
        ROS_WARN("Invalid data size for Absolute Mission Command");
        return;
    }
    
    AbsoluteMissionData mission_data;
    std::memcpy(&mission_data, data.data(), sizeof(AbsoluteMissionData));
    
    // Check password for mission validation
    if (mission_data.password != PASSWORD)
    {
        ROS_WARN("Invalid password provided, rejecting absolute mission request");
        return;
    }
    
    ROS_INFO_STREAM("Handling Absolute Mission: latitude=" << std::fixed << std::setprecision(8) 
                    << mission_data.latitude << ", longitude=" << mission_data.longitude 
                    << ", altitude=" << std::setprecision(3) << mission_data.altitude);
    
    osdk::MissionGoal goal;
    goal.relative = false;
    goal.abs_goal_position.latitude = mission_data.latitude;
    goal.abs_goal_position.longitude = mission_data.longitude;
    goal.abs_goal_position.altitude = mission_data.altitude;
    goal.max_distance = mission_data.max_distance;

    // const auto mission_succeeded = runMissionServer(goal, ac);
}

void handleRelativeMission(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(RelativeMissionData))
    {
        ROS_WARN("Invalid data size for Relative Mission Command");
        return;
    }
    
    RelativeMissionData mission_data;
    std::memcpy(&mission_data, data.data(), sizeof(RelativeMissionData));
    
    // Check password for mission validation
    if (mission_data.password != PASSWORD)
    {
        ROS_WARN("Invalid password provided, rejecting relative mission request");
        return;
    }
    
    ROS_INFO_STREAM("Handling Relative Mission: north=" << std::fixed << std::setprecision(3) 
                    << mission_data.north << ", east=" << mission_data.east 
                    << ", up=" << mission_data.up);
    
    osdk::MissionGoal goal;
    goal.relative = true;
    goal.rel_goal_position.x = mission_data.north;
    goal.rel_goal_position.y = mission_data.east;
    goal.rel_goal_position.z = mission_data.up;
    goal.max_distance = mission_data.max_distance;

    // const auto mission_succeeded = runMissionServer(goal, ac);
}

bool runMissionServer(const osdk::MissionGoal& goal, const actionlib::SimpleActionClient<osdk::MissionAction>& ac)
{
    ROS_INFO("Sending mission goal to action server");
    ac.sendGoal(goal);

    // Wait for the result
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
    
    if (!finished_before_timeout)
    {
        ROS_ERROR("Timed out waiting for action server to complete mission.");
        ac.cancelGoal();
        return false;
    }

    const auto result = ac.getResult();
    if (!result->success) 
    {
        ROS_ERROR("Mission failed: %s", result->message.c_str());
        return false;
    }

    ROS_INFO("Mission completed successfully: %s", result->message.c_str());
    return true;

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "mobile_data_subscriber");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<osdk::MissionAction> ac("mission_planner", true);
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Action server started, sending waypoints.");

    ros::Subscriber fromMobileDataSub = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, fromMobileDataSubCallback);
    
    ros::spin();
    return 0;
}


