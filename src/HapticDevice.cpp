#include "haptic_ros_driver/HapticDevice.h"

// haptic device API
#include "haptic_ros_driver/dhdc.h"
#include "std_msgs/Int8MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <libusb-1.0/libusb.h>

#include <tf2/LinearMath/Quaternion.h>


HapticDevice::HapticDevice(ros::NodeHandle & node, float loop_rate, bool set_force): loop_rate_(loop_rate)
{
    nh_ = node;

    dev_id_ = -2; // we set a value not in API defined range
    device_enabled_ = -1;

    set_force_ = set_force;

    for (int i = 0; i<3;i++) {
        position_[i] = 0.0;
        ori_entation_[i] = 0.0;
        rqt_force_[i] = 0.0;
    }

    for (int i =0; i<4; i++){
        orientation_[i] = 0.0;
    }

    button0_state_ = false;
    keep_alive_ = false;
    force_released_ = true;

    force_.resize(3);
    force_[0] = 0.0;
    force_[1] = 0.0;
    force_[2] = 0.0;

    SetForceLimit(10.0, 10.0, 10.0);

    // connect to hardware device
    device_count_ = dhdGetDeviceCount();

    // we only accept one haptic device.
    if ( device_count_ >= 1) {
        dev_id_ = dhdOpenID(0); // if open failed, we will get -1, and sucess with 0.
        if ( dev_id_ < 0) {
            ROS_INFO("error: handler device: %s\n", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    } else {
        ROS_INFO("No handler device find! %s\n", dhdErrorGetLastStr());
        device_enabled_ = false;
        return;
    }

    device_enabled_ =true;
}


HapticDevice::~HapticDevice()
{
    dev_id_ = -1;
    device_count_ = 0;
    keep_alive_ = false;
    if (dev_op_thread_)
    {
        dev_op_thread_->join();
    }
}

void HapticDevice::PublishHapticData()
{
    geometry_msgs::Vector3Stamped pos;
    pos.header.frame_id = ros::this_node::getName();
    pos.header.stamp = ros::Time::now();
    pos.vector.x = position_[0];
    pos.vector.y = position_[1];
    pos.vector.z = position_[2];

    geometry_msgs::Vector3 ori;
    // ori.header.frame_id = ros::this_node::getName();
    // ori.header.stamp = ros::Time::now();
    ori.x = ori_entation_[0];
    ori.y = ori_entation_[1];
    ori.z = ori_entation_[2];

    std_msgs::Int8MultiArray button_stat;
    button_stat.data.push_back(button0_state_);
    button_stat.data.push_back(button1_state_);

    geometry_msgs::Pose pose;
    pose.position.x = position_[0];
    pose.position.y = position_[1];
    pose.position.z = position_[2];
    pose.orientation.x = orientation_[0];
    pose.orientation.y = orientation_[1];
    pose.orientation.z = orientation_[2];
    pose.orientation.w = orientation_[3];

    geometry_msgs::Vector3 force;
    force.x = rqt_force_[0];
    force.y = rqt_force_[1];
    force.z = rqt_force_[2];

    position_pub_.publish(pos);
    orientation_pub_.publish(ori);
    pose_pub_.publish(pose);
    rqt_pub_.publish(force);
    button_state_pub_.publish(button_stat);
}

void HapticDevice::RegisterCallback()
{
    position_topic_ = "/haptic/position";
    buttons_topic_ = "/haptic/button_state";
    pose_topic_="/haptic/pose";
    ori_topic_ = "/haptic/ori";
    rqt_force_topic_="/rqt/force";

    position_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(position_topic_.c_str(),1);
    button_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>(buttons_topic_.c_str(), 1);
    orientation_pub_ = nh_.advertise<geometry_msgs::Vector3>(ori_topic_.c_str(),1);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>(pose_topic_.c_str(),1);
    rqt_pub_=nh_.advertise<geometry_msgs::Vector3>(rqt_force_topic_.c_str(),1);
    force_sub_ = nh_.subscribe("/haptic/force",10, &HapticDevice::ForceCallback, this);
}

void HapticDevice::ForceCallback(const geometry_msgs::Vector3ConstPtr &data)
{
    // wrapper force
    SetForce(data->x, data->y, data->z);
}

void HapticDevice::GetHapticDataRun()
{   // get and we will publish immediately


    double feed_force[3] = {0.0, 0.0, 0.0};
    double current_position[3] = {0.0, 0.0, 0.0};
    double current_orientation[3] = {0.0, 0.0, 0.0};
    double current_Quaternion[4] = {0.0, 0.0, 0.0, 0.0};

    while (ros::ok() && (keep_alive_ == true)) {

        if (device_count_ >= 1 && dev_id_ >= 0) {
                dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
                position_[0] = current_position[0];
                position_[1] = current_position[1];
                position_[2] = current_position[2];
                // ROS_INFO("position ");
                // ROS_INFO("x: %f", position_[0]);
                // ROS_INFO("y: %f", position_[1]);
                // ROS_INFO("z: %f", position_[2]);

                dhdGetOrientationRad(&current_orientation[0], &current_orientation[1], &current_orientation[2]);
                ori_entation_[0] = current_orientation[0];
                ori_entation_[1] = current_orientation[1];
                ori_entation_[2] = current_orientation[2];

                tf2::Quaternion myQuaternion;
                myQuaternion.setRPY(current_orientation[0], current_orientation[1], current_orientation[2]);
                current_Quaternion[0] = myQuaternion.getX();
                current_Quaternion[1] = myQuaternion.getY();
                current_Quaternion[2] = myQuaternion.getZ();
                current_Quaternion[3] = myQuaternion.getW();

                orientation_[0] = current_Quaternion[0];
                orientation_[1] = current_Quaternion[1];
                orientation_[2] = current_Quaternion[2];
                orientation_[3] = current_Quaternion[3];

                rqt_force_[0] = feed_force[0];
                rqt_force_[1] = feed_force[1];
                rqt_force_[2] = feed_force[2];

                // ROS_INFO("orientation");
                // ROS_INFO("x : %f", current_Quaternion[0]);
                // ROS_INFO("y : %f", current_Quaternion[1]);
                // ROS_INFO("z : %f", current_Quaternion[2]);
                // ROS_INFO("w : %f", current_Quaternion[3]);



                button0_state_ = dhdGetButton(0, dev_id_);
        }

        PublishHapticData();

        // apply force
        if (set_force_) {
            val_lock_.lock();

            // double dis = position_[0];
            // if(dis > 0){
            //     double stiffness = 500;
            //     feed_force[0] = -1 * dis * stiffness;
            //     feed_force[1] = 0.0;
            //     feed_force[2] = 0.0;
            // }

            // double range = pow(position_[0], 2.0) + pow(position_[1], 2) + pow(position_[2], 2);
            // if (range > 0.0015)
            // {
            //     double stiffness = 50;
            //     double f_x = position_[0] ;
            //     double f_y = position_[1] ;
            //     double f_z = position_[2] ;
            //     ROS_INFO("range : %f", range);

            //     feed_force[0] = -1 * f_x * stiffness;
            //     feed_force[1] = -1 * f_y * stiffness;
            //     feed_force[2] = -1 * f_z * stiffness;
            // }

            // else
            // {
            //     feed_force[0] = 0.0;
            //     feed_force[1] = 0.0;  
            //     feed_force[2] = 0.0;          
            // }
            feed_force[0] = force_[0];
            feed_force[1] = force_[1];
            feed_force[2] = force_[2];

            // std::cout <<"real put force_x" << feed_force[0] <<std::endl;
            // std::cout <<"real put force_y" << feed_force[1] <<std::endl;
            // std::cout <<"real put force_z" << feed_force[2] <<std::endl;
            dhdSetForce(feed_force[0], feed_force[1], feed_force[2]);
            val_lock_.unlock();
        }


        loop_rate_.sleep();
    }

}
double HapticDevice::ForceMapping(double x, int i)
{
    //force mapping
    if (i == 1){
        //f_x
        if( (-1) < x && x < 1)
            {
                return 0;
            }
        else 
            {
                return x/15;
            }   
    }

    else if (i == 2){
        //f_y
        if ((-1)< x && x<1)
        {
            return 0 ;
        }
        else
        {
            return  x/15;
        }
    }

    else if (i == 3){
        if ((-1)<x && x<1)
        {
            return 0 ;
        }
        else
        {
            return (-1) * x /15;
        }
    }
    else 
    { 
        return 0;
    }
    
}

void HapticDevice::SetForce(float x, float y, float z)
{
    double input_force[3] = {0.0, 0.0, 0.0};
    double put_force[3] = {0.0, 0.0, 0.0};

    if (set_force_)
    {
        val_lock_.lock();
        put_force[0] = static_cast<double>(x);
        put_force[1] = static_cast<double>(y);
        put_force[2] = static_cast<double>(z);

        // std::cout <<"force_x" << put_force[0] << std::endl;
        // std::cout <<"force_y" << put_force[1] << std::endl;
        // std::cout <<"force_z" << put_force[2] << std::endl;

        input_force[0] = HapticDevice::ForceMapping(put_force[0], 1);
        input_force[1] = HapticDevice::ForceMapping(put_force[1], 2);
        input_force[2] = HapticDevice::ForceMapping(put_force[2], 3);

        std::cout <<"After mapping force_x" << input_force[0] << std::endl;
        std::cout <<"After mapping force_y" << input_force[1] << std::endl;
        std::cout <<"After mapping force_z" << input_force[2] << std::endl;


        VerifyForceLimit(input_force, force_);
        force_released_ = false;
        val_lock_.unlock();
    }
}

void HapticDevice::SetForceLimit(double x, double y, double z)
{
    force_x_limit_ = x;
    force_y_limit_ = y;
    force_z_limit_ = z;
}


void HapticDevice::VerifyForceLimit(double input_force[], std::vector<double> & output)
{
    if (output.size() != 3) {
        output.resize(3);
    }

    output[0] = input_force[0];
    output[1] = input_force[1];
    output[2] = input_force[2];

    if (input_force[0] < -force_x_limit_) output[0] = -force_x_limit_;
    if (input_force[1] < -force_y_limit_) output[1] = -force_y_limit_;
    if (input_force[2] < -force_z_limit_) output[2] = -force_z_limit_;

    if (input_force[0] > force_x_limit_) output[0] = force_x_limit_;
    if (input_force[1] > force_y_limit_) output[1] = force_y_limit_;
    if (input_force[2] > force_z_limit_) output[2] = force_z_limit_;
}


void HapticDevice::Start()
{   
    if (!device_enabled_)
    {
        return; 
    }
    

    RegisterCallback();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    dev_op_thread_ = std::make_shared<boost::thread>(boost::bind(&HapticDevice::GetHapticDataRun, this));
    keep_alive_ = true;

    while (ros::ok() && (keep_alive_ == true)) {
        ros::Duration(0.001).sleep();
        // ROS_INFO("working in main loop");
        //  usleep(1000);
    }

    keep_alive_ = false;
    spinner.stop();
}
