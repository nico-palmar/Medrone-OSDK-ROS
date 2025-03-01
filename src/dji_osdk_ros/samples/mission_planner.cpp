#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <dji_osdk_ros/MissionAction.h>
#include <dji_osdk_ros/common_type.h>
#include <string>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/GetAvoidEnable.h>
#include <Eigen/Dense>
#include <dji_osdk_ros/MoveToWaypointAction.h>

namespace osdk = dji_osdk_ros;

namespace impulse_control
{
    
// Define the Action Server class
class MissionPlannerActionServer
{
public:
    MissionPlannerActionServer(std::string name) :
        as_(nh_, name, boost::bind(&MissionPlannerActionServer::executeMission, this, _1), false),
        action_name_(name),
        ac_("waypoint_control", true)
    {
        task_control_client_ = nh_.serviceClient<osdk::FlightTaskControl>("/flight_task_control");;
        enable_horizon_avoid_client_ = nh_.serviceClient<osdk::SetAvoidEnable>("/set_horizon_avoid_enable");
        enable_upward_avoid_client_ = nh_.serviceClient<osdk::SetAvoidEnable>("/set_upwards_avoid_enable");
        get_avoid_enable_client_ = nh_.serviceClient<osdk::GetAvoidEnable>("get_avoid_enable_status");
        obtain_ctrl_authority_client_ = nh_.serviceClient<osdk::ObtainControlAuthority>("obtain_release_control_authority");
        as_.start();
        ROS_INFO("Mission Planner Action Server started");
    }

    // Execute mission (takeoff -> waypoints -> landing)
    void executeMission(const osdk::MissionGoalConstPtr& goal)
    {
        // validate the mission
        if (!validMission(goal))
        {
            std::string err_msg = "The specified mission is not valid; must increase in altitude and only be " + std::to_string(goal->max_distance) + " km away at most";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }

        // obtain the control authority
        osdk::ObtainControlAuthority obtain_ctrl_authority;
  
        obtain_ctrl_authority.request.enable_obtain = true;
        obtain_ctrl_authority_client_.call(obtain_ctrl_authority);

        if (obtain_ctrl_authority.response.result == false)
        {
            std::string err_msg = "Failed to obtain autonomous control";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            result_.message = err_msg;
            as_.setAborted(result_, err_msg);
            return;
        }


        osdk::FlightTaskControl control_task;
        control_task.request.task = osdk::FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO("Takeoff request sending ...");
        // TODO: Figure out what happens if we send this but already took off
        task_control_client_.call(control_task);

        if (control_task.response.result == false)
        {
            std::string err_msg = "Takeoff failed!";
            ROS_ERROR("%s", err_msg.c_str());
            result_.success = false;
            as_.setAborted(result_, err_msg);
            return;
        }
        ROS_INFO("Takeoff successful, proceed to mission planner");

        ROS_INFO("Waiting for action server to start");
        ac_.waitForServer();
        ROS_INFO("Action server started, sending waypoints.");

        const auto waypoints = createWaypoints(goal);

        for (size_t i = 0; i < waypoints.size(); i++)
        {
            const auto waypoint = waypoints[i];
            if (!navigateToWaypoint(waypoint)) {
                ROS_ERROR("Failed to reach waypoint %ld", i + 1);
                result_.success = false;
                as_.setAborted(result_, "Navigation failed");
                return;
            }

            ROS_INFO("Reached waypoint %ld", i + 1);
            feedback_.n_waypoint = i+1;
            as_.publishFeedback(feedback_);

            // Check if the action has been preempted
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("Mission preempted");
                // consider returning to home here
                as_.setPreempted();
                ac_.cancelAllGoals();
                return;
            }
        }

        // TODO: add in landing later

        // Success!
        ROS_INFO("Mission completed successfully");
        result_.success = true;
        as_.setSucceeded(result_);
    }

private:

    std::vector<geometry_msgs::Point> createWaypoints(const osdk::MissionGoalConstPtr& goal)
    {
        // create waypoints in NEU (north east up) as relative goals (to the current location)
        // first move up to flying altitude
        geometry_msgs::Point pt_1, pt_2, pt_3;
        pt_1.x = 0.0;
        pt_1.y = 0.0;
        pt_1.z = FLYING_ALTITUDE_M;

        // then get the NED error and move the drone to (N, E, flight altitude)
        pt_2.x = static_cast<float>(goal_ned_error_.x());
        pt_2.y = static_cast<float>(goal_ned_error_.y());
        pt_2.z = 0.0;

        // drop the drone to (N, E, -D) = (N, E, U) relative position
        pt_3.x = 0.0;
        pt_3.y = 0.0;
        const auto desired_delta_z = -goal_ned_error_.z();
        // desired - current height will give the correct delta z to move the drone down by
        // TODO: Determine if the altitude should be user determined... or just hardset to some value such as (1 - FLYING_ALTITUDE_M)
        pt_3.z = desired_delta_z - FLYING_ALTITUDE_M;

        std::vector<geometry_msgs::Point> waypoints { pt_1, pt_2, pt_3 };
        return waypoints;
    }

    geographic_msgs::GeoPoint navSatFixtoGeoPoint(const sensor_msgs::NavSatFix& gps_pos)
    {
        // TODO: move to helper file later
        geographic_msgs::GeoPoint ret;
        ret.latitude = gps_pos.latitude;
        ret.longitude = gps_pos.longitude;
        ret.altitude = gps_pos.altitude;
        return ret;
    }

    Eigen::Vector3d getNEDError(const geographic_msgs::GeoPoint& current_position, const geographic_msgs::GeoPoint& target_position)
    {
        // TODO: move to helper file later
        // Convert GPS (WGS84) to UTM
        geodesy::UTMPoint ref_utm, target_utm;
        geodesy::fromMsg(current_position, ref_utm);
        geodesy::fromMsg(target_position, target_utm);

        // Calculate NED offsets
        Eigen::Vector3d ned_error { target_utm.northing - ref_utm.northing, target_utm.easting - ref_utm.easting, -(target_position.altitude - current_position.altitude) };
        // ROS_DEBUG_STREAM("The NED Error is N: " << ned_error.x() << ", E: " << ned_error.y() << " D: " << ned_error.z());
        return ned_error;
    }

    geographic_msgs::GeoPoint localToGlobalGoal(const geographic_msgs::GeoPoint& curr_pos, const geometry_msgs::Point& rel_goal_pos)
    {
        // TODO: move to helper file later
        // Convert the anchor GPS to a geodesy UTM point
        geodesy::UTMPoint curr_utm;
        geodesy::fromMsg(curr_pos, curr_utm);

        // Create a new UTM point with the offset applied
        geodesy::UTMPoint utm_target;
        utm_target.easting  = curr_utm.easting + rel_goal_pos.x;  // East direction
        utm_target.northing = curr_utm.northing + rel_goal_pos.y; // North direction
        utm_target.altitude = curr_utm.altitude + rel_goal_pos.z; // Altitude change
        utm_target.zone     = curr_utm.zone;
        utm_target.band     = curr_utm.band;

        // Convert back to GPS (lat, lon, alt)
        const geographic_msgs::GeoPoint target_gps = geodesy::toMsg(utm_target);

        return target_gps;
    }

    bool validMission(const osdk::MissionGoalConstPtr& goal, const float gps_timeout_s = 5.0)
    {
        // validate altitude isn't lower than the starting position
        const auto msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", nh_, ros::Duration(gps_timeout_s));
        const auto curr_pos = navSatFixtoGeoPoint(*msg);

        geographic_msgs::GeoPoint goal_pos;
        if (goal->relative)
        {
            goal_pos = localToGlobalGoal(curr_pos, goal->rel_goal_position);
        }
        else
        {
            goal_pos = goal->abs_goal_position;
        }

        goal_ned_error_ = getNEDError(curr_pos, goal_pos);

        const auto goal_distance = goal_ned_error_.norm();

        if (goal_distance > goal->max_distance)
        {
            return false;
        }

        // check that the drone isn't moving down from  the current position
        // assume valid missions must go up to avoid crashing
        if (goal_ned_error_.z() > 0)
        {
            // down error is positive; drone wants to go down
            return false;
        }
        return true;
    }

    bool navigateToWaypoint(const geometry_msgs::Point& waypoint)
    {
        osdk::MoveToWaypointGoal goal;
        goal.relative = true;
        goal.rel_goal_position.x = waypoint.x;
        goal.rel_goal_position.y = waypoint.y;
        goal.rel_goal_position.z = waypoint.z;

        ac_.sendGoal(goal);
        const auto finished_before_timeout = ac_.waitForResult(ros::Duration(SINLGE_WAYPOINT_TIMEOUT_S));

        if (!finished_before_timeout)
        {
            ROS_ERROR("Timed out waiting for action server to reach waypoint");
            return false;
        }

        const auto waypoint_result = ac_.getResult();
        return waypoint_result->success;
    }

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<osdk::MissionAction> as_;
    std::string action_name_;

    osdk::MissionFeedback feedback_;
    osdk::MissionResult result_;

    actionlib::SimpleActionClient<osdk::MoveToWaypointAction> ac_;

    ros::ServiceClient task_control_client_;
    ros::ServiceClient enable_horizon_avoid_client_;
    ros::ServiceClient enable_upward_avoid_client_;
    ros::ServiceClient get_avoid_enable_client_;
    ros::ServiceClient obtain_ctrl_authority_client_;

    Eigen::Vector3d goal_ned_error_;

    // primitive mission planner information
    const int N_WAYPOINTS { 3 };
    // the flight altitude of the drone
    const double FLYING_ALTITUDE_M { 20.0 };

    const double SINLGE_WAYPOINT_TIMEOUT_S { 60.0 };

};
    

} // impulse_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_planner_action_server");
    impulse_control::MissionPlannerActionServer server("mission_planner");
    ros::spin();
    return 0;
}
