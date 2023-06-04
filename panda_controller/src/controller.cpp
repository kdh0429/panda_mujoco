#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc) : dc_(dc)
{
    dc_.sim_mode_ = "position";
    
    right_arm_.id_ = "right_arm";
    left_arm_.id_ = "left_arm";
    robots_.push_back(&right_arm_);
    robots_.push_back(&left_arm_);

    // RBDL
    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout<<"Robot Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &right_arm_.robot_model_, false, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &left_arm_.robot_model_, false, false);
}

PandaController::~PandaController()
{

}

void PandaController::compute()
{
    ros::Rate r(2000);
    while(ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (!is_init_)
            {
                for (auto &robot: robots_)
                {
                    robot->q_.resize(DOF);
                    robot->q_.setZero();
                    robot->q_dot_.resize(DOF);
                    robot->q_dot_.setZero();
                    robot->q_dot_zero_.resize(DOF);
                    robot->q_dot_zero_.setZero();
                    robot->effort_.resize(DOF);
                    robot->effort_.setZero();
                    robot->control_input_.resize(DOF);
                    robot->control_input_.setZero();

                    robot->j_temp_.resize(6, DOF);
                    robot->j_temp_.setZero();
                    robot->j_.resize(6, DOF);
                    robot->j_.setZero();

                    robot->A_.resize(DOF, DOF);
                    robot->A_.setZero();
                    robot->g_.resize(DOF);
                    robot->g_.setZero();
                }

                is_init_ = true;
            }

            m_dc_.lock();
            sim_time_ = dc_.sim_time_;
            right_arm_.q_ = dc_.q_.head(DOF);
            left_arm_.q_ = dc_.q_.tail(DOF);
            right_arm_.q_dot_ = dc_.q_dot_.head(DOF);
            left_arm_.q_dot_ = dc_.q_dot_.tail(DOF);
            right_arm_.effort_ = dc_.effort_.head(DOF);
            left_arm_.effort_ = dc_.effort_.tail(DOF);
            right_arm_.ft_.head(3) = dc_.force_.head(3);
            right_arm_.ft_.tail(3) = dc_.torque_.head(3);
            left_arm_.ft_.head(3) = dc_.force_.tail(3);
            left_arm_.ft_.tail(3) = dc_.torque_.tail(3);
            m_dc_.unlock();

            updateKinematicsDynamics();
            computeControlInput();
        }
        r.sleep();
    }
}

void PandaController::updateKinematicsDynamics()
{
    for (auto &robot: robots_)
    {
        static const int BODY_ID = robot->robot_model_.GetBodyId("panda_link8");

        // Kinematoics
        robot->x_.translation().setZero();
        robot->x_.linear().setZero();
        robot->x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
        robot->x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot->robot_model_, robot->q_, BODY_ID, true).transpose();

        robot->j_temp_.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), robot->j_temp_, true);
        robot->j_.setZero();
        for (int i = 0; i<2; i++)
        {
            robot->j_.block<3, 7>(i * 3, 0) = robot->j_temp_.block<3, 7>(3 - i * 3, 0);
        }  

        robot->x_dot_ = robot->j_ * robot->q_dot_;

        RigidBodyDynamics::NonlinearEffects(robot->robot_model_, robot->q_, robot->q_dot_zero_, robot->g_);
    }
}

void PandaController::computeControlInput()
{
    for (auto &robot: robots_)
    {
        if (robot->id_ == "right_arm")
        {
            for (int i = 0; i <DOF; i++)
            {
                robot->control_input_(i) = robot->g_(i);
            }
        }
        else if (robot->id_ == "left_arm")
        {
            for (int i = 0; i < DOF; i++)
            {
                robot->control_input_(i) = M_PI / 6.0;
            }
        }
    }

    m_ci_.lock();
    dc_.control_input_.head(DOF) = right_arm_.control_input_;
    dc_.control_input_.tail(DOF) = left_arm_.control_input_;
    m_ci_.unlock();
}