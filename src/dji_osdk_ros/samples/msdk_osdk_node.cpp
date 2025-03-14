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
#include <std_msgs/UInt32.h>
#include <future>
#include <dji_osdk_ros/ObtainControlAuthority.h>

namespace osdk = dji_osdk_ros;

using CommandHandler = std::function<void(const std::vector<uint8_t>&)>;

namespace impulse_comms
{
class MobileCommandHandler
{
public:
    MobileCommandHandler(std::string name) :
        nh_(),
        ac_("mission_planner", true)
    {
        // Initialize the action client
        ROS_INFO("Waiting for action server to start...");
        ac_.waitForServer();
        ROS_INFO("Action server started, ready to receive commands.");

        // Subscribe to mobile data
        fromMobileDataSub_ = nh_.subscribe("dji_osdk_ros/from_mobile_data", 10, 
                                          &MobileCommandHandler::fromMobileDataSubCallback, this);

        drop_trigger_pub_ = nh_.advertise<std_msgs::UInt32>("drop_trigger", 1000);

        obtain_ctrl_authority_client_ = nh_.serviceClient<osdk::ObtainControlAuthority>("obtain_release_control_authority");
        
        command_handlers_ = {
            std::bind(&MobileCommandHandler::handleCommandA, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleCommandB, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleDropTrigger, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleAbsoluteMission, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleRelativeMission, this, std::placeholders::_1)
        };
    }

    // Define data structures for incoming requests
    struct __attribute__((packed)) CommandAData
    {
        double test_val;
    };

    struct __attribute__((packed)) CommandBData
    {
        uint8_t flag;
        bool trigger;
    };

    struct __attribute__((packed)) TriggerDropData
    {
        uint32_t password;
        uint8_t drop_flag;
    };

    struct __attribute__((packed)) RelativeMissionData
    {
        uint32_t password;
        double north;
        double east;
        double up;
        double max_distance;
    };

    struct __attribute__((packed)) AbsoluteMissionData
    {
        uint32_t password;
        double latitude;
        double longitude;
        double altitude;
        double max_distance;
    };

private:
    ros::NodeHandle nh_;
    ros::Subscriber fromMobileDataSub_;
    ros::Publisher drop_trigger_pub_;
    actionlib::SimpleActionClient<osdk::MissionAction> ac_;
    std::vector<CommandHandler> command_handlers_;
    ros::ServiceClient obtain_ctrl_authority_client_;
    
    // Constants
    const uint32_t MSDK_PASSWORD { 46000636 };
    const uint32_t UART_PASSWORD { 18922601 };
    const uint8_t DROP_FLAG { 42 };

    void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData) {
        ROS_INFO("Recived mobile data");
        if (fromMobileData->data.empty()) {
            ROS_INFO("Received empty data from mobile");
            return;
        }

        // Handle command IDno instance of overloaded function "std::async" matches the argument list
        uint8_t command_id = static_cast<uint8_t>(fromMobileData->data[0]);
        ROS_INFO_STREAM("Received command ID: " << static_cast<int>(command_id));

        if (command_id >= command_handlers_.size())
        {
            ROS_WARN_STREAM("Unknown command ID received: " << command_id);
            return;
        }

        if (!testOSDKActivation())
        {
            ROS_ERROR("Control authority is dead; ignoring message");
            return;
        }

        CommandHandler handler = command_handlers_[command_id];
        std::vector<uint8_t> payload(fromMobileData->data.begin() + 1, fromMobileData->data.end());

        handler(payload);
    }

    bool callAuthorityService(osdk::ObtainControlAuthority &srv) {
        return obtain_ctrl_authority_client_.call(srv);
    }

    bool testOSDKActivation()
    {
        // Call service in a separate thread
        osdk::ObtainControlAuthority obtain_ctrl_authority;
        obtain_ctrl_authority.request.enable_obtain = true;
        std::future<bool> result = std::async(std::launch::async, &MobileCommandHandler::callAuthorityService, this, std::ref(obtain_ctrl_authority));

        // Wait up to 10 seconds for a response
        if (result.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            if (result.get()) {
                ROS_INFO("Service response: %d", obtain_ctrl_authority.response.result);
                return true;
            } else {
                ROS_ERROR("Service call failed.");
                return false;
            }
        } else {
            ROS_ERROR("Service call timed out after 10 seconds.");
            return false;
        }
    }

    void handleCommandA(const std::vector<uint8_t>& data) {
        if (data.size() < sizeof(CommandAData)) 
        {
            ROS_WARN("Invalid data size for Command A");
            return;
        }
        CommandAData cmdA;
        std::memcpy(&cmdA, data.data(), sizeof(CommandAData));
        ROS_INFO_STREAM("Handling Command A: param1=" << std::fixed << std::setprecision(6) 
                        << static_cast<double>(cmdA.test_val));
    }

    void handleCommandB(const std::vector<uint8_t>& data) {
        if (data.size() < sizeof(CommandBData)) {
            ROS_WARN("Invalid data size for Command B");
            return;
        }
        CommandBData cmdB;
        std::memcpy(&cmdB, data.data(), sizeof(CommandBData));
        ROS_INFO_STREAM("Handling Command B: flag=" << static_cast<int>(cmdB.flag) 
                        << ", bool trigger=" << cmdB.trigger);
    }

    void handleDropTrigger(const std::vector<uint8_t>& data) {
        if (data.size() < sizeof(TriggerDropData)) {
            ROS_WARN_STREAM("Invalid data size for Drop Trigger Command");
            return;
        }
        TriggerDropData trigger_drop_cmd;
        std::memcpy(&trigger_drop_cmd, data.data(), sizeof(TriggerDropData));
        
        // Check the password fields for drop triggering
        if (!(trigger_drop_cmd.password == MSDK_PASSWORD && trigger_drop_cmd.drop_flag == DROP_FLAG)) {
            ROS_WARN_STREAM("Invalid drop combination provided, rejecting drop request. PWD " << trigger_drop_cmd.password << " and flag " << static_cast<int>(trigger_drop_cmd.drop_flag));
            return;
        }
        ROS_INFO("Drop command received; triggering over UART");

        std_msgs::UInt32 uart_pwd;
        uart_pwd.data = UART_PASSWORD;
        drop_trigger_pub_.publish(uart_pwd);
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
        if (mission_data.password != MSDK_PASSWORD)
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

        runMissionServer(goal);
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
        if (mission_data.password != MSDK_PASSWORD)
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

        runMissionServer(goal);
    }

    void runMissionServer(const osdk::MissionGoal& goal)
    {
        ROS_INFO("Sending mission goal to action server");
        ac_.sendGoal(goal);

        // Wait for the result
        // TODO: add preemption to the missions (not working)
        const auto finished_before_timeout = ac_.waitForResult(ros::Duration(200.0));
        
        if (!finished_before_timeout)
        {
            ROS_ERROR("Timed out waiting for action server to complete mission.");
            ac_.cancelGoal();
            return;
        }

        const auto result = ac_.getResult();
        if (!result->success)
        {
            ROS_ERROR("Mission failed: %s", result->message.c_str());
            return;
        }

        ROS_INFO("Mission completed successfully: %s", result->message.c_str());
    }
};

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mobile_command_handler");
    impulse_comms::MobileCommandHandler handler("mobile_command_handler");
    ros::spin();
    return 0;
}
