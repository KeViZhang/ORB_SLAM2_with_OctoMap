/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <time.h>
#include <stdio.h>
using namespace std;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped CurrentPose;
ofstream f,f_Cam;
int Nsec_OFF = -1;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/**
 * @brief SetTargetPose
 * @param TargetPose
 */
void SetTargetPose(geometry_msgs::PoseStamped *TargetPose)
{
    TargetPose->pose.position.x = CurrentPose.pose.position.x;
    TargetPose->pose.position.y = CurrentPose.pose.position.y;
    TargetPose->pose.position.z = CurrentPose.pose.position.z + 0.4;
    TargetPose->pose.orientation = CurrentPose.pose.orientation;
    //ROS_INFO("Set target pose!");
    cout << endl << "Set target pose!" << endl << *TargetPose << endl;
}
void SaveMAVPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgpose )
{
    CurrentPose = *msgpose;
    f << msgpose->header.stamp.sec - Nsec_OFF << "." << msgpose->header.stamp.nsec << " " <<
         setprecision(9) << msgpose->pose.position.x << " " << msgpose->pose.position.y << " " << msgpose->pose.position.z << " " <<
         msgpose->pose.orientation.x << " " << msgpose->pose.orientation.y << " " << msgpose->pose.orientation.z << " " << msgpose->pose.orientation.w << endl;
}

void SaveCameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& CameraPose)
{
    f_Cam << CameraPose->header.stamp.sec - Nsec_OFF << "." << CameraPose->header.stamp.nsec << " " << setprecision(9) <<
         CameraPose->pose.position.x << " " << CameraPose->pose.position.y << " " << CameraPose->pose.position.z << " " <<
         CameraPose->pose.orientation.x << " " << CameraPose->pose.orientation.y << " " << CameraPose->pose.orientation.z << " " << CameraPose->pose.orientation.w << endl;
}
/**
 * @brief getfilename
 * @return The filename of the MAV pose record, named as the time(yyyy-mm-dd-hh-mm-ss);
 */
string getfilename(string ss)
{
    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    ostringstream oss;
    oss << ss << timeinfo->tm_year + 1900 << "-" << timeinfo->tm_mon + 1 << "-" << timeinfo->tm_mday <<
           "-" << timeinfo->tm_hour << "-" << timeinfo->tm_min << "-" << timeinfo->tm_sec << ".txt";
    string strfilename = oss.str();
    return strfilename;
}

int main(int argc, char **argv)
{
    string filename = getfilename("mav-");
    f.open(filename.c_str());
    f << fixed;
    cout << endl << "Saving MAV pose to " << filename << " ..." << endl;

    string filenameCameraPose = getfilename("camera-");
    f_Cam.open(filenameCameraPose.c_str());
    f_Cam << fixed;
    cout << endl << "Saving Camera pose to " << filenameCameraPose << " ..." << endl;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Time current_time;
    Nsec_OFF = current_time.now().sec;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, SaveMAVPoseCallback);
    ros::Subscriber camera_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 10, SaveCameraPoseCallback);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped TargetPose;
    std_msgs::Header header_msg;
    size_t seq_id_ = 0;

    header_msg.frame_id = "/base_link";
    header_msg.seq = seq_id_;
    header_msg.stamp = current_time.now();

    TargetPose.header = header_msg;
    TargetPose.pose.position.x = 0;
    TargetPose.pose.position.y = 0;
    TargetPose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(TargetPose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();  

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");                
                SetTargetPose(&TargetPose);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    SetTargetPose(&TargetPose);
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        header_msg.seq = ++seq_id_;
        header_msg.stamp = current_time.now();
        TargetPose.header = header_msg;
        local_pos_pub.publish(TargetPose);

        ros::spinOnce();
        rate.sleep();
    }
    f.close();
    cout << endl << "MAV pose saved!" << endl;

    f_Cam.close();
    cout << endl << "Camera pose saved!" << endl;
    return 0;
}


/*
template<typename T> std::string StringUtils::toString(const T &t)
{
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

template<typename T>
T StringUtils::fromstring(const std::string& s)
{
    std::istringstream stream(s);
    T t;
    stream >> t;
    return t;
}
*/
