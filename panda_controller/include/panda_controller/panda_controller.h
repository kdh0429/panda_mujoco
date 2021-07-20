#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>

#include "mujoco_ros_msgs/JointSet.h"

#include "panda_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

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
        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
        Eigen::VectorXd q_dot_zero_;
        Eigen::VectorXd effort_;

        Eigen::VectorXd control_input_;

        RigidBodyDynamics::Model robot_;
        Eigen::VectorXd g_;   
};