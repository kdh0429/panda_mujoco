#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>

#include "mujoco_ros_msgs/JointSet.h"

#include "panda_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "panda_controller/util.h"

#define DOF 7

class PandaController{
    public:
        PandaController(ros::NodeHandle &nh, DataContainer &dc);
        ~PandaController();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();

    private:
        std::mutex m_dc_;
        std::mutex m_ci_;
        DataContainer &dc_;

        bool is_init_ = false;

        double sim_time_ = 0.0;

        // Robot State
        struct RobotState
        {
            std::string id_;

            Eigen::Vector7d q_;
            Eigen::Vector7d q_init_;
            Eigen::Vector7d q_dot_;
            Eigen::Vector7d q_dot_zero_;
            Eigen::Vector7d effort_;

            // Kinematics & Dynamics
            RigidBodyDynamics::Model robot_model;
            RigidBodyDynamics::Model &robot_model_ = robot_model;

            Eigen::MatrixXd A_;
            Eigen::VectorXd g_;

            Eigen::Isometry3d x_;
            Eigen::Vector6d x_dot_;
            Eigen::MatrixXd j_temp_;
            Eigen::MatrixXd j_;

            Eigen::Vector7d control_input_;
        };
        RobotState right_arm_;
        RobotState left_arm_;

        std::vector<RobotState*> robots_; 
};