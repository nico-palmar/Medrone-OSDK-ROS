#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dji_osdk_ros/Mission.h>
#include <dji_osdk_ros/common_type.h>
#include <string>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

namespace osdk = dji_osdk_ros;

namespace impulse_control
{
    
// Define the Action Server class
class MissionPlannerActionServer
{
public:
    MissionPlannerActionServer(std::string name) :
        as_(nh_, name, boost::bind(&MissionPlannerActionServer::executeMission, this, _1), false),
        action_name_(name)
    {
        task_control_client_ = nh_.serviceClient<FlightTaskControl>("/flight_task_control");;
        enable_horizon_avoid_client_ = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
        enable_upward_avoid_client_ = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
        get_avoid_enable_client_ = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
        obtain_ctrl_authority_client_ = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");

        gps_sub_ = nh_.subscribe("dji_osdk_ros/gps_position", 10, &MissionPlannerActionServer::gpsPosCallback, this);

        as_.start();
        ROS_INFO("Mission Planner Action Server started");
    }

    // Execute mission (takeoff -> waypoints -> landing)
    void executeMission(const osdk::MissionGoalConstPtr& goal)
    {

        // validate the mission
        if (!validMission(goal))

        // obtain the control authority
        osdk::ObtainControlAuthority obtain_ctrl_authority;
  
        obtain_ctrl_authority.request.enable_obtain = true;
        obtain_ctrl_authority_client.call(obtain_ctrl_authority);

        if (obtain_ctrl_authority.response.result == false)
        {
            std::string err_msg = "Failed to obtain autonomous control";
            ROS_ERROR(err_msg);
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
        }



        // Step 1: Takeoff
        if (!takeoff()) {
            ROS_ERROR("Takeoff failed!");
            result_.success = false;
            as_.setAborted(result_, "Takeoff failed");
            return;
        }

        // Step 2: Waypoint Navigation
        for (size_t i = 0; i < goal->waypoints.poses.size(); ++i)
        {
            geometry_msgs::Pose waypoint = goal->waypoints.poses[i];
            ROS_INFO("Navigating to waypoint %ld", i + 1);

            if (!navigateToWaypoint(waypoint)) {
                ROS_ERROR("Failed to reach waypoint %ld", i + 1);
                result_.success = false;
                as_.setAborted(result_, "Navigation failed");
                return;
            }

            // Send feedback
            feedback_.current_position = waypoint; // Example: send last successful position
            as_.publishFeedback(feedback_);

            // Check if the action has been preempted
            if (as_.isPreemptRequested()) {
                ROS_WARN("Mission preempted");
                as_.setPreempted();
                return;
            }
        }

        // Step 3: Land if requested
        if (goal->auto_land)
        {
            if (!land()) {
                ROS_ERROR("Landing failed!");
                result_.success = false;
                as_.setAborted(result_, "Landing failed");
                return;
            }
        }

        // Success!
        ROS_INFO("Mission completed successfully");
        result_.success = true;
        as_.setSucceeded(result_);
    }

    // Takeoff function
    bool takeoff()
    {
        ROS_INFO("Taking off...");
        ros::Duration(5.0).sleep(); // Simulate takeoff
        return true;
    }

    // Navigate to a waypoint
    bool navigateToWaypoint(const geometry_msgs::Pose &waypoint)
    {
        ROS_INFO("Moving to (X: %.2f, Y: %.2f, Z: %.2f)",
                    waypoint.position.x, waypoint.position.y, waypoint.position.z);
        ros::Duration(3.0).sleep(); // Simulate movement
        return true;
    }

    // Landing function
    bool land()
    {
        ROS_INFO("Landing...");
        ros::Duration(5.0).sleep(); // Simulate landing
        return true;
    }


private:

    bool validMission(const osdk::MissionGoalConstPtr& goal)
    {
        // validate altitude isn't lower
    }

    void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<osdk::MissionAction> as_;
    std::string action_name_;

    // Feedback & result messages
    osdk::MissionFeedback feedback_;
    osdk::MissionResult result_;
    ros::ServiceClient task_control_client_;
    ros::ServiceClient enable_horizon_avoid_client_;
    ros::ServiceClient enable_upward_avoid_client_;
    ros::ServiceClient get_avoid_enable_client_;
    ros::ServiceClient obtain_ctrl_authority_client_;

    ros::Subscriber gps_sub_;

};
    

} // impulse_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_planner_action_server");
    impulse_control::MissionPlannerActionServer server("mission_planner");
    ros::spin();
    return 0;
}
