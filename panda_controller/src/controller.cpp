#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc) : dc_(dc)
{
    dc_.sim_mode_ = "torque";
    
    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout<<"Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, false, false);
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
                q_.resize(dc_.num_dof_);
                q_.setZero();
                q_dot_.resize(dc_.num_dof_);
                q_dot_.setZero();
                q_dot_zero_.resize(dc_.num_dof_);
                q_dot_zero_.setZero();
                effort_.resize(dc_.num_dof_);
                effort_.setZero();
                control_input_.resize(dc_.num_dof_);
                control_input_.setZero();

                g_.resize(dc_.num_dof_);
                g_.setZero();

                is_init_ = true;
            }

            m_dc_.lock();
            sim_time_ = dc_.sim_time_;
            q_ = dc_.q_;
            q_dot_ = dc_.q_dot_;
            effort_ = dc_.effort_;
            m_dc_.unlock();

            updateKinematicsDynamics();
            computeControlInput();
        }
        r.sleep();
    }
}

void PandaController::updateKinematicsDynamics()
{
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_zero_, g_);
}

void PandaController::computeControlInput()
{
    for (int i = 0; i <dc_.num_dof_; i++)
    {
        control_input_(i) = g_(i);
    }

    m_ci_.lock();
    dc_.control_input_ = control_input_;
    m_ci_.unlock();
}