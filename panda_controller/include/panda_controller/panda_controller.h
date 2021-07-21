#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>

#include "mujoco_ros_msgs/JointSet.h"

#include "panda_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class PandaController{
    public:
        PandaController(ros::NodeHandle &nh, DataContainer &dc);
        ~PandaController();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();
        void initMoveit();
        void setMoveitObstables();
        void generateRandTraj();

    private:
        std::mutex m_dc_;
        std::mutex m_ci_;
        DataContainer &dc_;

        bool is_init_ = false;

        double sim_time_ = 0.0;

        // Robot State
        Eigen::VectorXd q_;
        Eigen::VectorXd q_dot_;
        Eigen::VectorXd q_dot_zero_;
        Eigen::VectorXd effort_;

        Eigen::VectorXd control_input_;

        // Kinematics & Dynamics
        RigidBodyDynamics::Model robot_;
        Eigen::VectorXd g_;   

        // Moveit
        inline static const std::string PLANNING_GROUP="panda_arm";
        moveit::planning_interface::MoveGroupInterface move_group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        moveit::planning_interface::MoveGroupInterface::Plan random_plan_;

        Eigen::VectorXd q_limit_u_;
        Eigen::VectorXd q_limit_l_;
};