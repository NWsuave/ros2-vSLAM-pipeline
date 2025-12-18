/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ros2_orb_slam3/common.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 10);
path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orbslam3/path", 10);

tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


path_msg_.header.frame_id = map_frame_;

}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
	// --- DEBUG: confirm callback runs ---
	// DEBUG (optional)
RCLCPP_INFO(this->get_logger(), "Img_callback firing");

// Convert to world pose (camera in map)
Sophus::SE3f Twc = Tcw.inverse();
rclcpp::Time stamp(msg.header.stamp);
// ===== Publish dynamic map->odom using SLAM + TF odom->camera_link =====

// 1) T_map_camera from ORB (Twc)
Sophus::SE3f T_map_cam = Twc;

// 2) Lookup current odom->camera_link from TF
geometry_msgs::msg::TransformStamped odom_to_cam_msg;
try {
    // Use the same timestamp as the image/SLAM result
    odom_to_cam_msg = tf_buffer_->lookupTransform(
        odom_frame_,        // target frame (parent)
        camera_frame_,      // source frame (child)
        rclcpp::Time(msg.header.stamp),
        rclcpp::Duration::from_seconds(0.05)
    );
} catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed (odom->camera_link): %s", ex.what());
    return; // can't compute map->odom without this
}

// Convert odom->camera_link to Sophus SE3
const auto &t_msg = odom_to_cam_msg.transform.translation;
const auto &r_msg = odom_to_cam_msg.transform.rotation;

Eigen::Quaternionf q_odom_cam((float)r_msg.w, (float)r_msg.x, (float)r_msg.y, (float)r_msg.z);
q_odom_cam.normalize();
Eigen::Vector3f p_odom_cam((float)t_msg.x, (float)t_msg.y, (float)t_msg.z);


Sophus::SE3f T_odom_cam(q_odom_cam.toRotationMatrix(), p_odom_cam);

// 3) Compute T_map_odom = T_map_cam * inverse(T_odom_cam)
Sophus::SE3f T_map_odom = T_map_cam * T_odom_cam.inverse();

// 4) Broadcast map->odom TF
Eigen::Vector3f p_map_odom = T_map_odom.translation();
Eigen::Quaternionf q_map_odom(T_map_odom.rotationMatrix());
q_map_odom.normalize();

geometry_msgs::msg::TransformStamped map_to_odom_msg;
map_to_odom_msg.header.stamp = rclcpp::Time(msg.header.stamp);
map_to_odom_msg.header.frame_id = map_frame_;
map_to_odom_msg.child_frame_id = odom_frame_;
map_to_odom_msg.transform.translation.x = p_map_odom.x();
map_to_odom_msg.transform.translation.y = p_map_odom.y();
map_to_odom_msg.transform.translation.z = p_map_odom.z();
map_to_odom_msg.transform.rotation.x = q_map_odom.x();
map_to_odom_msg.transform.rotation.y = q_map_odom.y();
map_to_odom_msg.transform.rotation.z = q_map_odom.z();
map_to_odom_msg.transform.rotation.w = q_map_odom.w();

tf_broadcaster_->sendTransform(map_to_odom_msg);

// Publish PoseStamped of camera in map (for RViz)
Eigen::Vector3f p_map_cam = T_map_cam.translation();
Eigen::Quaternionf q_map_cam(T_map_cam.rotationMatrix());
q_map_cam.normalize();

geometry_msgs::msg::PoseStamped pose_msg;
pose_msg.header.stamp = stamp;
pose_msg.header.frame_id = map_frame_;
pose_msg.pose.position.x = p_map_cam.x();
pose_msg.pose.position.y = p_map_cam.y();
pose_msg.pose.position.z = p_map_cam.z();
pose_msg.pose.orientation.x = q_map_cam.x();
pose_msg.pose.orientation.y = q_map_cam.y();
pose_msg.pose.orientation.z = q_map_cam.z();
pose_msg.pose.orientation.w = q_map_cam.w();
pose_pub_->publish(pose_msg);

// Publish Path (cap length)
path_msg_.header.stamp = stamp;
path_msg_.header.frame_id = map_frame_;
path_msg_.poses.push_back(pose_msg);
if (path_msg_.poses.size() > max_path_len_) {
  path_msg_.poses.erase(path_msg_.poses.begin(),
                        path_msg_.poses.begin() + (path_msg_.poses.size() - max_path_len_));
}
path_pub_->publish(path_msg_);

RCLCPP_INFO(this->get_logger(), "Publishing map->odom and pose/path");
}
