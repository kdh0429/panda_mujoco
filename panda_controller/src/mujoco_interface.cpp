#include "panda_controller/mujoco_interface.h"

MujocoInterface::MujocoInterface(ros::NodeHandle &nh, DataContainer &dc): dc_(dc)
{
    if (ros::this_node::getNamespace() == "/master")
    {
        mujoco_sim_status_sub_ = nh.subscribe("/mujoco_ros_interface/master/sim_status", 1, &MujocoInterface::simStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
        mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/master/joint_set", 100);
    }
    else if (ros::this_node::getNamespace() == "/slave")
    {
        mujoco_sim_status_sub_ = nh.subscribe("/mujoco_ros_interface/slave/sim_status", 1, &MujocoInterface::simStatusCallback, this, ros::TransportHints().tcpNoDelay(true));
        mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/slave/joint_set", 100);
    }
}

MujocoInterface::~MujocoInterface()
{

}

void MujocoInterface::stateUpdate()
{
    ros::Rate r(2000);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}

void MujocoInterface::simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg)
{
    if (is_first_callback)
    {
        dc_.num_dof_ = msg->name.size();

        dc_.sim_time_ = msg->time;

        dc_.q_.resize(dc_.num_dof_);
        dc_.q_dot_.resize(dc_.num_dof_);
        dc_.effort_.resize(dc_.num_dof_);
        dc_.q_.setZero();
        dc_.q_dot_.setZero();
        dc_.effort_.setZero();

        for (int j=0; j<dc_.num_dof_; j++)
        {
            dc_.q_[j] = msg->position[j];
            dc_.q_dot_[j] = msg->velocity[j];
            dc_.effort_[j] = msg->effort[j];
        }

        dc_.control_input_.resize(dc_.num_dof_);
        dc_.control_input_.setZero();

        mujoco_joint_set_msg_.torque.resize(dc_.num_dof_);
        mujoco_joint_set_msg_.position.resize(dc_.num_dof_);

        is_first_callback = false;
        dc_.is_first_callback = is_first_callback;
    }
    else
    {
        for (int j=0; j<dc_.num_dof_; j++)
        {
            dc_.q_[j] = msg->position[j];
            dc_.q_dot_[j] = msg->velocity[j];
            dc_.effort_[j] = msg->effort[j];
        }

        for (int i=0; i< msg->sensor.size(); i++)
        {
            if (msg->sensor[i].name == "Force_sensor")
                {
                    for (int j=0; j<3; j++)
                        dc_.force_[j] = msg->sensor[i].data[j];
                }
        }
        dc_.sim_time_ = msg->time;
    }
}

void MujocoInterface::sendCommand(int control_mode)
{
    ros::Rate r(2000);
    while (ros::ok())
    {
        if (!is_first_callback)
        {                
            if (control_mode == 1)
            {
                if (dc_.sim_mode_ == "torque")
                {
                    mujoco_joint_set_msg_.MODE = 1;

                    for (int i = 0; i < dc_.num_dof_; i++)
                    {
                        mujoco_joint_set_msg_.torque[i] = dc_.control_input_[i];
                    }
                }
                else
                {
                    std::cout << "COMMAND DISMATCH! -- mujoco mode : positon, controller mode : torque" << std::endl;
                }
            }
            else if (control_mode == 0)
            {
                if (dc_.sim_mode_ == "position")
                {
                    mujoco_joint_set_msg_.MODE = 0;

                    for (int i = 0; i < dc_.num_dof_; i++)
                    {
                        mujoco_joint_set_msg_.position[i] = dc_.control_input_[i];
                    }
                }
                else
                {
                    std::cout << "COMMAND DISMATCH! -- mujoco mode : torque, controller mode : position" << std::endl;
                }
            }

            mujoco_joint_set_msg_.header.stamp = ros::Time::now();
            mujoco_joint_set_msg_.time = dc_.sim_time_;
            mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
        }

        r.sleep();
    }
}