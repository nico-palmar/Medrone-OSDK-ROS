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
#include <atomic>

namespace osdk = dji_osdk_ros;

using CommandHandler = std::function<void(const std::vector<uint8_t>&)>;

namespace impulse_comms
{
class MobileCommandHandler
{
public:
    MobileCommandHandler(std::string name) :
        nh_(),
        ac_("mission_planner", true),
        authority_check_thread_running_(false)
    {
        // Initialize the action client
        ROS_INFO("Waiting for action server to start...");
        ac_.waitForServer();
        ROS_INFO("Action server started, ready to receive commands.");

        // Subscribe to mobile data
        fromMobileDataSub_ = nh_.subscribe("dji_osdk_ros/from_mobile_data", 100,
                                        &MobileCommandHandler::fromMobileDataSubCallback, this);

        drop_trigger_pub_ = nh_.advertise<std_msgs::UInt32>("drop_trigger", 1000);

        obtain_ctrl_authority_client_ = nh_.serviceClient<osdk::ObtainControlAuthority>("obtain_release_control_authority");

        // reset the variables for next time
        authority_check_in_progress_.store(false);
        has_authority_.store(true);
        
        command_handlers_ = {
            std::bind(&MobileCommandHandler::handleCommandA, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleCommandB, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleDropTrigger, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleAbsoluteMission, this, std::placeholders::_1),
            std::bind(&MobileCommandHandler::handleRelativeMission, this, std::placeholders::_1)
        };

        authority_check_thread_running_.store(true);
        authority_check_thread_ = std::thread(&MobileCommandHandler::authorityCheckLoop, this);
    }

    ~MobileCommandHandler()
    {
        // Signal thread to stop and wait for it
        authority_check_thread_running_.store(false);
        if (authority_check_thread_.joinable()) {
            authority_check_thread_.join();
        }
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
    // ros::Timer cancel_mission_timer_;
    // ros::Timer authority_check_timer_;
    std::atomic<bool> authority_check_in_progress_;
    std::atomic<bool> has_authority_;
    std::thread authority_check_thread_;
    std::atomic<bool> authority_check_thread_running_;
    std::atomic<bool> mission_active_;
    std::mutex authority_mutex_;
    
    const uint32_t MSDK_PASSWORD { 46000636 };
    const uint32_t UART_PASSWORD { 18922601 };
    const uint8_t DROP_FLAG { 42 };
    const int OSDK_AUTHORITY_WAIT_TIME_S { 10 };
    const double CHECK_CANCEL_MISSION_PERIOD_S { 1 };
    const double CHECK_AUTHORITY_TIMER_S { 1 };

    void authorityCheckLoop()
    {
        ros::Rate rate(1.0 / CHECK_AUTHORITY_TIMER_S);
        while (authority_check_thread_running_.load() && ros::ok())
        {
            if (mission_active_.load())
            {
                auto should_cancel = false;
                {
                    // Minimize the critical section
                    std::lock_guard<std::mutex> lock(authority_mutex_);
                    if (!authority_check_in_progress_.exchange(true))
                    {
                        bool has_authority = osdkHasAuthority();
                        has_authority_.store(has_authority);
                        should_cancel = !has_authority;
                        authority_check_in_progress_.store(false);
                    }
                }

                if (should_cancel)
                {
                    ROS_ERROR("Lost authority, cancelling mission");
                    ac_.cancelGoal();
                    mission_active_.store(false);
                    // Wait a bit to ensure cancellation is processed
                    ros::Duration(0.5).sleep();
                }
            }
            rate.sleep();
        }
    }

    void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData)
    {
        ROS_INFO("Recived mobile data");
        if (fromMobileData->data.empty()) {
            ROS_INFO("Received empty data from mobile");
            return;
        }

        // Handle command ID
        uint8_t command_id = static_cast<uint8_t>(fromMobileData->data[0]);
        ROS_INFO_STREAM("Received command ID: " << static_cast<int>(command_id));

        if (command_id >= command_handlers_.size())
        {
            ROS_WARN_STREAM("Unknown command ID received: " << command_id);
            return;
        }

        CommandHandler handler = command_handlers_[command_id];
        std::vector<uint8_t> payload(fromMobileData->data.begin() + 1, fromMobileData->data.end());

        handler(payload);
    }

    bool callAuthorityService(osdk::ObtainControlAuthority &srv)
    {
        return obtain_ctrl_authority_client_.call(srv);
    }

    bool osdkHasAuthority()
    {
        osdk::ObtainControlAuthority obtain_ctrl_authority;
        obtain_ctrl_authority.request.enable_obtain = true;
        std::future<bool> result = std::async(std::launch::async, &MobileCommandHandler::callAuthorityService, this, std::ref(obtain_ctrl_authority));

        if (result.wait_for(std::chrono::seconds(OSDK_AUTHORITY_WAIT_TIME_S)) != std::future_status::ready)
        {
            ROS_ERROR_STREAM("Service call timed out after " << OSDK_AUTHORITY_WAIT_TIME_S << " seconds.");
            return false;
        }
        if (!result.get())
        {
            ROS_ERROR("Service call failed.");
            return false;
        }
        if (!obtain_ctrl_authority.response.result)
        {
            ROS_ERROR("Service call has a result of false");
            return false;
        }

        // otherwise, response is true, osdk has authority
        return true;
    }

    void handleCommandA(const std::vector<uint8_t>& data)
    {
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

    void handleCommandB(const std::vector<uint8_t>& data)
    {
        if (data.size() < sizeof(CommandBData)) {
            ROS_WARN("Invalid data size for Command B");
            return;
        }
        CommandBData cmdB;
        std::memcpy(&cmdB, data.data(), sizeof(CommandBData));
        ROS_INFO_STREAM("Handling Command B: flag=" << static_cast<int>(cmdB.flag) 
                        << ", bool trigger=" << cmdB.trigger);
    }

    void handleDropTrigger(const std::vector<uint8_t>& data)
    {
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

    void handleAbsoluteMission(const std::vector<uint8_t>& data)
    {
        if (data.size() < sizeof(AbsoluteMissionData))
        {
            ROS_WARN("Invalid data size for Absolute Mission Command");
            return;
        }

        if (!osdkHasAuthority())
        {
            ROS_ERROR("Control authority is dead; ignoring message");
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

    void handleRelativeMission(const std::vector<uint8_t>& data)
    {
        if (data.size() < sizeof(RelativeMissionData)) 
        {
            ROS_WARN("Invalid data size for Relative Mission Command");
            return;
        }

        if (!osdkHasAuthority())
        {
            ROS_ERROR("Control authority is dead; ignoring message");
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

    void missionCompleteCallback(const actionlib::SimpleClientGoalState& state, const osdk::MissionResultConstPtr& result)
    {
        // authority_check_timer_.stop();
        // cancel_mission_timer_.stop();

        // reset the variables for next time
        authority_check_in_progress_.store(false);
        // assume we have authority on reset to not cancel a mission by accident
        has_authority_.store(true);

        if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Mission failed with state: %s, message: %s",
                state.toString().c_str(), result->message.c_str());
            return;
        }
        ROS_INFO("Mission completed successfully: %s", result->message.c_str());
    }

    void activeCallback()
    {
        ROS_INFO("Goal just went active");
        mission_active_.store(true);
        // Start a timer to periodically update the cancel mission status
        // authority_check_timer_ = nh_.createTimer(ros::Duration(CHECK_AUTHORITY_TIMER_S),
        //     [this](const ros::TimerEvent&) {
        //         if (authority_check_in_progress_.exchange(true))
        //         {
        //             // authority check is already occuring; skip this one
        //             ROS_ERROR("SKIPPING OSDK AUTHROITY CHECK");
        //             return;
        //         }

        //         // check the authority
        //         // ROS_ERROR("SKIPPING OSDK AUTHROITY CHECK");
        //         const auto has_authority = osdkHasAuthority();
        //         has_authority_.store(has_authority);

        //         if (!has_authority)
        //         {
        //             ROS_ERROR("Cancel condition met, cancelling mission");
        //             ac_.cancelGoal();
        //             ros::Duration(0.5).sleep();
        //             authority_check_timer_.stop();
        //             // cancel_mission_timer_.stop();
        //             authority_check_in_progress_.store(false);
        //             has_authority_.store(true);
        //             // wait for the goal cancelling to go through
        //         }

        //         authority_check_in_progress_.store(false);
        //     }
        // );

        // Start a timer to periodically check if we should cancel the mission
        // cancel_mission_timer_ = nh_.createTimer(ros::Duration(CHECK_CANCEL_MISSION_PERIOD_S),
        //     [this](const ros::TimerEvent&) {
        //         // ROS_ERROR("CANCEL MISSION CHECKING");
        //         if (has_authority_.load() == false)
        //         {
        //             ROS_ERROR("Cancel condition met, cancelling mission");
        //             ac_.cancelGoal();
        //             authority_check_timer_.stop();
        //             cancel_mission_timer_.stop();
        //             authority_check_in_progress_.store(false);
        //             has_authority_.store(true);
        //         }
        //     }
        // );
    }

    void feedbackCallback(const osdk::MissionFeedbackConstPtr& feedback)
    {
        // Keep this in case we want to log any data for feedback during missions
    }

    void runMissionServer(const osdk::MissionGoal& goal)
    {
        // handle mission preemption
        if (ac_.getState().state_ == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // Cancel the current mission
            ROS_WARN("Preempting current mission with new request");
            ac_.cancelGoal();
            // Small delay to ensure cancellation is processed
            ros::Duration(0.1).sleep();
        }

        ROS_INFO("Sending mission goal to action server");
        ac_.sendGoal(goal, std::bind(&MobileCommandHandler::missionCompleteCallback, this,
            std::placeholders::_1, std::placeholders::_2),
            std::bind(&MobileCommandHandler::activeCallback, this),
            std::bind(&MobileCommandHandler::feedbackCallback, this, std::placeholders::_1));
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
