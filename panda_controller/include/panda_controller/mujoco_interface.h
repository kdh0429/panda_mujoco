#include <ros/ros.h>
#include "panda_controller/data_container.h"
#include "mujoco_ros_msgs/SimStatus.h"
#include "mujoco_ros_msgs/JointSet.h"

#ifndef MujocoInterface_H
#define MujocoInterface_H
class MujocoInterface
{
    public:
        MujocoInterface(ros::NodeHandle &nh, DataContainer &dc, bool is_master);
        ~MujocoInterface();
        void stateUpdate();
        void simStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg);
        void sendCommand(int control_mode);

    private:
        DataContainer &dc_;
        
        ros::Subscriber mujoco_sim_status_sub_;
        ros::Publisher mujoco_joint_set_pub_;

        bool is_first_callback = true;

        mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;
};
#endif