#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc) : dc_(dc)
{
    dc_.sim_mode_ = "torque";
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
                effort_.resize(dc_.num_dof_);
                effort_.setZero();
                control_input_.resize(dc_.num_dof_);
                control_input_.setZero();

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

}

void PandaController::computeControlInput()
{
    control_input_(0) = 50;
    control_input_(1) = 50;
    control_input_(2) = 50;
    control_input_(3) = 50;
    control_input_(4) = 50;
    control_input_(5) = 50;
    control_input_(6) = 50;

    m_ci_.lock();
    dc_.control_input_ = control_input_;
    m_ci_.unlock();
}