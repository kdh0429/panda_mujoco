#include "panda_controller/slave_panda_controller.h"

SlavePandaController::SlavePandaController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc)
{
    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";
    master_state_sub_ = nh.subscribe("/mujoco_ros_interface/master/sim_status", 1, &SlavePandaController::masterStatusCallback, this, ros::TransportHints().tcpNoDelay(true));

    // RBDL
    urdf_name_ = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout<<"Model name: " << urdf_name_ <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name_.c_str(), &robot_, false, false);
    
    // Logging
    if (is_write_)
    {
        writeFile.open("/home/kim/ssd2/data.csv", std::ofstream::out | std::ofstream::app);
        writeFile << std::fixed << std::setprecision(8);
    }
    
    // Torch
    loadNetwork();

    ros::Duration(2.0).sleep();
}

SlavePandaController::~SlavePandaController()
{

}

void SlavePandaController::masterStatusCallback(const mujoco_ros_msgs::SimStatusConstPtr &msg)
{
    if(is_init_)
    {
        for (int j=0; j<dc_.num_dof_; j++)
        {
            q_desired_(j) = msg->position[j];
            q_dot_desired_(j) = msg->velocity[j];
        }
    }
}


void SlavePandaController::compute()
{
    ros::Rate r(hz_);
    while(ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (!is_init_)
            {
                 std::cout <<"First Callback"<<std::endl;
                // Robot State
                q_.setZero();
                q_dot_.setZero();
                effort_.setZero();

                q_mode_init_.resize(dc_.num_dof_);
                q_mode_init_.setZero();
                q_dot_mode_init_.resize(dc_.num_dof_);
                q_dot_mode_init_.setZero();

                j_temp_.resize(6, dc_.num_dof_);
                j_temp_.setZero();
                j_.resize(6, dc_.num_dof_);
                j_.setZero();

                x_.linear().setZero();
                x_.translation().setZero();
                x_dot_.resize(6);
                x_dot_.setZero();
                x_dot_desired_.resize(6);
                x_dot_desired_.setZero();
            
                // Control
                q_dot_desired_.resize(dc_.num_dof_);
                q_dot_desired_.setZero();
                q_desired_.resize(dc_.num_dof_);
                q_desired_.setZero();

                kp.resize(dc_.num_dof_, dc_.num_dof_);
                kp.setZero();
                kv.resize(dc_.num_dof_, dc_.num_dof_);
                kv.setZero();
                kp_task_.resize(6,6);
                kp_task_.setZero();
                kv_task_.resize(6,6);
                kv_task_.setZero();

                for (int i = 0; i < dc_.num_dof_; i++)
                {
                    kp(i,i) = 2500;
                    kv(i,i) = 100;
                }
                for (int i = 0; i < 6; i++)
                {
                    kp_task_(i,i) = 4900;
                    kv_task_(i,i) = 140;
                }

                control_input_.resize(dc_.num_dof_);
                control_input_.setZero();
                control_input_filtered_.setZero();

                non_linear_.resize(dc_.num_dof_);
                A_.resize(dc_.num_dof_, dc_.num_dof_);
                A_.setZero();
                C_.resize(dc_.num_dof_, dc_.num_dof_);
                C_.setZero();
                Lambda_.resize(6,6);
                Lambda_.setZero();

                // Torch
                std::fill(ring_buffer_, ring_buffer_+num_seq*num_features*num_joint, 0);
                estimated_ext_torque_LSTM_.resize(dc_.num_dof_);
                estimated_ext_torque_LSTM_.setZero();
                measured_ext_torque_.resize(dc_.num_dof_);
                measured_ext_torque_.setZero();
                estimated_ext_force_SOSML_.resize(dc_.num_dof_);
                estimated_ext_force_SOSML_.setZero();
                estimated_ext_force_ESO_.resize(dc_.num_dof_);
                estimated_ext_force_ESO_.setZero();
                estimated_ext_force_HOFTO_.resize(dc_.num_dof_);
                estimated_ext_force_HOFTO_.setZero();

                estimated_ext_force_.resize(6);
                estimated_ext_force_.setZero();
                estimated_ext_force_pre_.resize(6);
                estimated_ext_force_pre_.setZero();
                estimated_ext_force_init_.resize(6);
                estimated_ext_force_init_.setZero();
                measured_ext_force_.resize(6);
                measured_ext_force_.setZero();

                output_scaling << 124.64, 82.809, 122.57, 81.091, 11.176, 6.7371, 6.8228;

                // Sliding Mode Momentum Observer
                p_.setZero();
                p_hat_.setZero();
                p_tilde_.setZero();
                p_tilde_sign_.setZero();

                g_.resize(dc_.num_dof_);
                g_.setZero();
                
                sigma_.setZero();

                T1_.setZero();
                T2_.setZero();
                S1_.setZero();
                S2_.setZero();

                double s1 = 160;
                double s2 = 2500;
                T1_.diagonal() << 2.6*sqrt(s1), 2.6*sqrt(s1), 2.6*sqrt(s1), 2.6*sqrt(s1), 2.6*sqrt(s1), 2.6*sqrt(s1), 2.6*sqrt(s1);
                T2_.diagonal() << 2*sqrt(s2), 2*sqrt(s2), 2*sqrt(s2), 2*sqrt(s2), 2*sqrt(s2), 2*sqrt(s2), 2*sqrt(s2);
                S1_.diagonal() << s1, s1, s1, s1, s1, s1, s1;
                S2_.diagonal() << s2, s2, s2, s2, s2, s2, s2;

                estimated_ext_torque_SOSML_.setZero();

                // Extended State Observer
                x1_.setZero();
                x1_hat_.setZero();
                x1_tilde_.setZero();
                x2_hat_.setZero();
                x3_hat_.setZero();

                eta1_ = 10.0;
                eta2_ = 1.0;
                epsilon_ = 0.01;

                A_ESO_.resize(dc_.num_dof_, dc_.num_dof_);
                A_ESO_.setZero();
                C_ESO_.setZero();
                g_ESO_.resize(dc_.num_dof_);
                g_ESO_.setZero();

                estimated_ext_torque_ESO_.setZero();

                // High-Order Finite Time Observer
                z1_.setZero(); 
                z2_.setZero();
                z3_.setZero();
                z4_.setZero();

                x1_z1_diff_.setZero();

                L1_.diagonal() << 22.3607,22.3607,21.6265,20.8090,20.8090,20.8090,20.8090; 
                L2_.diagonal() << 22.101,22.101,21.1419,20.083,20.083,20.083,20.083;
                L3_.diagonal() << 30,30,28.0624,25.9808,25.9808,25.9808,25.9808;
                L4_.diagonal() << 440,440,385,330,330,330,330;

                L1_.diagonal() = L1_.diagonal()*0.01;
                L2_.diagonal() = L2_.diagonal()*0.01;
                L3_.diagonal() = L3_.diagonal()*0.01;
                L4_.diagonal() = L4_.diagonal()*0.01;

                x1_HOFTO_.setZero();
                x2_HOFTO_.setZero();

                m2_ = 1 + (2-1) * (-2/9); // 7/9
                m3_ = 1 + (3-1) * (-2/9); // 5/7
                m4_ = 1 + (4-1) * (-2/9); // 3/7
                m5_ = 1 + (5-1) * (-2/9); // 1/7

                A_HOFTO_.resize(dc_.num_dof_, dc_.num_dof_);
                A_HOFTO_.setZero();
                
                non_linear_HOFTO_.resize(dc_.num_dof_);
                non_linear_HOFTO_.setZero();

                estimated_ext_torque_HOFTO_.setZero();

                init_time_ = ros::Time::now().toSec();

                is_init_ = true;
            }

            if (is_init_)
            {
                cur_time_= ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                q_ = dc_.q_;
                q_dot_ = dc_.q_dot_;
                effort_ = dc_.effort_;
                m_dc_.unlock();

                updateKinematicsDynamics();

                // computeSOSML();
                // computeESO();
                // computeHOFTO();

                writeBuffer();
                computeTrainedModel();
                computeExtForce();

                if (is_write_)
                {
                    logData(); // When to log data is important
                }

                computeControlInput();  

                printData();                

                pre_time_ = cur_time_;
            }

            ros::spinOnce();
            r.sleep();
        }
    }
}

void SlavePandaController::updateKinematicsDynamics()
{
    static const int BODY_ID = robot_.GetBodyId("panda_link7");
    x_.translation().setZero();
    x_.linear().setZero();

    x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
    x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot_, q_, BODY_ID, true).transpose();
    
    j_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(robot_, q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), j_temp_, true);

    j_.setZero();
    for (int i = 0; i<2; i++)
	{
		j_.block<3, 7>(i * 3, 0) = j_temp_.block<3, 7>(3 - i * 3, 0);
	}    
    x_dot_ = j_ * q_dot_;

    non_linear_.setZero();
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_, non_linear_);

    g_.setZero();
    Eigen::Vector7d q_dot_zero;
    q_dot_zero.setZero();
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_zero, g_);

    A_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, A_, true);
    C_ = getC(q_, q_dot_);
    Lambda_ = (j_ * A_.inverse() * j_.transpose()).inverse();
}

void SlavePandaController::computeControlInput()
{
    control_input_ = A_*(kv*(q_dot_desired_ - q_dot_) + kp * (q_desired_ - q_)) + non_linear_;


    for (int i = 0; i < dc_.num_dof_; i++)
    {
        control_input_filtered_(i) = lowPassFilter(control_input_(i), control_input_filtered_(i), 1/hz_, 20);
    }
    
    dc_.control_input_ = control_input_;
}

void SlavePandaController::writeBuffer()
{
    if (int(cur_time_*100) != int(pre_time_*100))
    {
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i] = 2*(q_(i)-min_theta_)/(max_theta_-min_theta_) - 1;
            ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i + 1] = 2*(q_dot_(i)-min_theta_dot_)/(max_theta_dot_-min_theta_dot_) - 1;
            ring_buffer_control_input_[ring_buffer_idx_*num_joint + i] = 2*(control_input_filtered_(i)+output_scaling(i))/(output_scaling(i)+output_scaling(i)) - 1;
        }
        
        ring_buffer_idx_++;
        if (ring_buffer_idx_ == num_seq)
            ring_buffer_idx_ = 0;
    }
}

void SlavePandaController::logData()
{
    if (int(cur_time_*100) != int(pre_time_*100))
    {
        writeFile << cur_time_ << "\t";

        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << q_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << q_dot_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << q_desired_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << q_dot_desired_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << q_ddot_desired_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << control_input_filtered_(i) << "\t";
        }

        // int cur_idx = 0;
        // if (ring_buffer_idx_ == 0)
        //     cur_idx = num_seq - 1;
        // else
        //     cur_idx = ring_buffer_idx_ - 1;
        
        // for (int i=0; i< dc_.num_dof_; i++)
        // {
        //     writeFile << ring_buffer_control_input_[cur_idx*num_joint + i]  << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << network_output_share_(i) << "\t";
        // }
        
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << estimated_ext_torque_LSTM_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << measured_ext_torque_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << estimated_ext_torque_SOSML_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << estimated_ext_torque_ESO_(i) << "\t";
        // }

        // writeFile << f_d_x_ << "\t" << estimated_ext_force_(0) << "\t" << measured_ext_force_(0) << "\t" << -dc_.force_(0);

        // for (int i = 0; i < 3; i++)
        // {
        //     writeFile << measured_ext_force_(i) << "\t";
        // }
        // for (int i = 0; i < 3; i++)
        // {
        //     writeFile << estimated_ext_force_(i) << "\t";
        // }
        // for (int i = 0; i < 3; i++)
        // {
        //     writeFile << estimated_ext_force_SOSML_(i) << "\t";
        // }

        writeFile << "\n";
    }
}

void SlavePandaController::computeExtForce()
{
    Eigen::Matrix<double, 7, 1> ext;
    ext.setZero();
    ext = (A_*dc_.effort_ + non_linear_) - control_input_;

    Eigen::Vector7d lstm_output_copy;
    lstm_output_copy = network_output_share_;

    int cur_idx = 0;
    if (ring_buffer_idx_ == 0)
        cur_idx = num_seq - 1;
    else
        cur_idx = ring_buffer_idx_ - 1;

    for (int i= 0; i < dc_.num_dof_; i++)
    {
        estimated_ext_torque_LSTM_(i) = ring_buffer_control_input_[cur_idx*num_joint + i] - lstm_output_copy(i);
        estimated_ext_torque_LSTM_(i) = estimated_ext_torque_LSTM_(i) * output_scaling(i);

        if (i < 4)
        {
            measured_ext_torque_(i) = -(ext(i) + dc_.q_dot_(i) * 100.0);
            estimated_ext_torque_SOSML_(i) = estimated_ext_torque_SOSML_(i) - dc_.q_dot_(i) * 100.0;
            estimated_ext_torque_ESO_(i) = estimated_ext_torque_ESO_(i) - dc_.q_dot_(i) * 100.0;
            estimated_ext_torque_HOFTO_(i) = estimated_ext_torque_HOFTO_(i) - dc_.q_dot_(i) * 100.0;
        }
        else
        {
            measured_ext_torque_(i) = -(ext(i) + dc_.q_dot_(i) * 10.0);
            estimated_ext_torque_SOSML_(i) = estimated_ext_torque_SOSML_(i) - dc_.q_dot_(i) * 10.0;
            estimated_ext_torque_ESO_(i) = estimated_ext_torque_ESO_(i) - dc_.q_dot_(i) * 10.0;
            estimated_ext_torque_HOFTO_(i) = estimated_ext_torque_HOFTO_(i) - dc_.q_dot_(i) * 10.0;
        }
    }
    
    Eigen::Matrix<double, 6,6> I;
    I.setIdentity();
    j_dyn_cons_inv_T_ = (j_*A_.inverse()*j_.transpose() + 0.001*I).inverse() * j_ * A_.inverse();

    estimated_ext_force_ = j_dyn_cons_inv_T_*estimated_ext_torque_LSTM_;

    estimated_ext_force_SOSML_ = j_dyn_cons_inv_T_*estimated_ext_torque_SOSML_;
    estimated_ext_force_ESO_ = j_dyn_cons_inv_T_*estimated_ext_torque_ESO_;
    estimated_ext_force_HOFTO_ = j_dyn_cons_inv_T_*estimated_ext_torque_HOFTO_;

    measured_ext_force_ = j_dyn_cons_inv_T_*measured_ext_torque_;
}

void SlavePandaController::printData()
{ 
    if (int(cur_time_*10) != int(pre_time_*10))
    {
        std::cout <<"FT Measured Force: " << dc_.force_(0) <<std::setw(12)<< dc_.force_(1) <<std::setw(12)<< dc_.force_(2) <<std::endl;
        std::cout <<"Dynamics Measured Force: " << measured_ext_force_(0) <<std::setw(12)<< measured_ext_force_(1) <<std::setw(12)<< measured_ext_force_(2) <<std::endl << std::endl;

        std::cout <<"Estimated Force LSTM: " << estimated_ext_force_(0) <<std::setw(12)<< estimated_ext_force_(1) <<std::setw(12)<< estimated_ext_force_(2) <<std::setw(12)<< estimated_ext_force_(3) <<std::setw(12)<< estimated_ext_force_(4) <<std::setw(12)<< estimated_ext_force_(5) <<std::endl;
        std::cout <<"Estimated Force SOSML: " << estimated_ext_force_SOSML_(0) <<std::setw(12)<< estimated_ext_force_SOSML_(1) <<std::setw(12)<< estimated_ext_force_SOSML_(2) <<std::endl;
        std::cout <<"Estimated Force ESO: " << estimated_ext_force_ESO_(0) <<std::setw(12)<< estimated_ext_force_ESO_(1) <<std::setw(12)<< estimated_ext_force_ESO_(2) <<std::endl;
        std::cout <<"Estimated Force HOFTO: " << estimated_ext_force_HOFTO_(0) <<std::setw(12)<< estimated_ext_force_HOFTO_(1) <<std::setw(12)<< estimated_ext_force_HOFTO_(2) <<std::endl << std::endl;

        std::cout <<"Dynamics Measured Ext: " << std::setw(12) << measured_ext_torque_(0) << std::setw(12) << measured_ext_torque_(1) << std::setw(12) << measured_ext_torque_(2) <<std::setw(12)<< measured_ext_torque_(3) << std::setw(12) << measured_ext_torque_(4)<< std::setw(12) << measured_ext_torque_(5) << std::setw(12) << measured_ext_torque_(6) <<std::endl;
        std::cout <<"LSTM Ext: " << std::setw(12) << estimated_ext_torque_LSTM_(0) << std::setw(12) << estimated_ext_torque_LSTM_(1) << std::setw(12) << estimated_ext_torque_LSTM_(2) <<std::setw(12)<< estimated_ext_torque_LSTM_(3) << std::setw(12) << estimated_ext_torque_LSTM_(4)<< std::setw(12) << estimated_ext_torque_LSTM_(5) << std::setw(12) << estimated_ext_torque_LSTM_(6) <<std::endl;
        std::cout <<"SOSML Ext: " << estimated_ext_torque_SOSML_(0) <<std::setw(12)<< estimated_ext_torque_SOSML_(1) << std::setw(12) << estimated_ext_torque_SOSML_(2) << std::setw(12) << estimated_ext_torque_SOSML_(3) << std::setw(12) << estimated_ext_torque_SOSML_(4) << std::setw(12) << estimated_ext_torque_SOSML_(5) << std::setw(12) << estimated_ext_torque_SOSML_(6) <<std::endl;
        std::cout <<"ESO Ext: " << estimated_ext_torque_ESO_(0) <<std::setw(12)<< estimated_ext_torque_ESO_(1) <<std::setw(12)<< estimated_ext_torque_ESO_(2) <<std::setw(12) << estimated_ext_torque_ESO_(3) <<std::setw(12)<< estimated_ext_torque_ESO_(4) <<std::setw(12)<< estimated_ext_torque_ESO_(5) <<std::setw(12)<< estimated_ext_torque_ESO_(6) <<std::endl;
        std::cout <<"HOFTO Ext: " << estimated_ext_torque_HOFTO_(0) <<std::setw(12)<< estimated_ext_torque_HOFTO_(1) <<std::setw(12)<< estimated_ext_torque_HOFTO_(2) <<std::setw(12) << estimated_ext_torque_HOFTO_(3) <<std::setw(12)<< estimated_ext_torque_HOFTO_(4) <<std::setw(12)<< estimated_ext_torque_HOFTO_(5) <<std::setw(12)<< estimated_ext_torque_HOFTO_(6) <<std::endl;
        

                std::cout<<"dt: " << cur_time_ - pre_time_ << std::endl;

        std::cout << std::endl;
    }
}

void SlavePandaController::loadNetwork()
{
    std::ifstream file[12];
    file[0].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_0_weight.txt", std::ios::in);
    file[1].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_0_bias.txt", std::ios::in);
    file[2].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_2_weight.txt", std::ios::in);
    file[3].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_2_bias.txt", std::ios::in);
    file[4].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_4_weight.txt", std::ios::in);
    file[5].open("/home/kim/panda_ws/src/panda_controller/model/backward_network_4_bias.txt", std::ios::in);
    file[6].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_0_weight.txt", std::ios::in);
    file[7].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_0_bias.txt", std::ios::in);
    file[8].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_2_weight.txt", std::ios::in);
    file[9].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_2_bias.txt", std::ios::in);
    file[10].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_4_weight.txt", std::ios::in);
    file[11].open("/home/kim/panda_ws/src/panda_controller/model/forward_network_4_bias.txt", std::ios::in);

    if(!file[0].is_open())
    {
        std::cout<<"Can not find the weight file"<<std::endl;
    }

    float temp;
    int row = 0;
    int col = 0;

    while(!file[0].eof() && row != backward_W0.rows())
    {
        file[0] >> temp;
        if(temp != '\n')
        {
            backward_W0(row, col) = temp;
            col ++;
            if (col == backward_W0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[1].eof() && row != backward_b0.rows())
    {
        file[1] >> temp;
        if(temp != '\n')
        {
            backward_b0(row, col) = temp;
            col ++;
            if (col == backward_b0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[2].eof() && row != backward_W2.rows())
    {
        file[2] >> temp;
        if(temp != '\n')
        {
            backward_W2(row, col) = temp;
            col ++;
            if (col == backward_W2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[3].eof() && row != backward_b2.rows())
    {
        file[3] >> temp;
        if(temp != '\n')
        {
            backward_b2(row, col) = temp;
            col ++;
            if (col == backward_b2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[4].eof() && row != backward_W4.rows())
    {
        file[4] >> temp;
        if(temp != '\n')
        {
            backward_W4(row, col) = temp;
            col ++;
            if (col == backward_W4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[5].eof() && row != backward_b4.rows())
    {
        file[5] >> temp;
        if(temp != '\n')
        {
            backward_b4(row, col) = temp;
            col ++;
            if (col == backward_b4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[6].eof() && row != forward_W0.rows())
    {
        file[6] >> temp;
        if(temp != '\n')
        {
            forward_W0(row, col) = temp;
            col ++;
            if (col == forward_W0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[7].eof() && row != forward_b0.rows())
    {
        file[7] >> temp;
        if(temp != '\n')
        {
            forward_b0(row, col) = temp;
            col ++;
            if (col == forward_b0.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[8].eof() && row != forward_W2.rows())
    {
        file[8] >> temp;
        if(temp != '\n')
        {
            forward_W2(row, col) = temp;
            col ++;
            if (col == forward_W2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[9].eof() && row != forward_b2.rows())
    {
        file[9] >> temp;
        if(temp != '\n')
        {
            forward_b2(row, col) = temp;
            col ++;
            if (col == forward_b2.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[10].eof() && row != forward_W4.rows())
    {
        file[10] >> temp;
        if(temp != '\n')
        {
            forward_W4(row, col) = temp;
            col ++;
            if (col == forward_W4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[11].eof() && row != forward_b4.rows())
    {
        file[11] >> temp;
        if(temp != '\n')
        {
            forward_b4(row, col) = temp;
            col ++;
            if (col == forward_b4.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
}

void SlavePandaController::computeBackwardDynamicsModel()
{
    Eigen::Matrix<float, num_seq*num_features*num_joint, 1> backNetInput;
    
    backNetInput << condition_, state_;
    
    backward_layer1_ = backward_W0 * backNetInput + backward_b0;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (backward_layer1_(i) < 0)
            backward_layer1_(i) = 0.0;
    }

    backward_layer2_ = backward_W2 * backward_layer1_ + backward_b2;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (backward_layer2_(i) < 0)
            backward_layer2_(i) = 0.0;
    }

    backward_network_output_ = backward_W4 * backward_layer2_ + backward_b4;
} 

void SlavePandaController::computeForwardDynamicsModel()
{
    Eigen::Matrix<float, (num_seq-1)*num_features*num_joint + num_joint, 1> forwardNetInput;
    forwardNetInput << condition_, input_;
    
    forward_layer1_ = forward_W0 * forwardNetInput + forward_b0;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (forward_layer1_(i) < 0)
            forward_layer1_(i) = 0.0;
    }

    forward_layer2_ = forward_W2 * forward_layer1_ + forward_b2;
    for (int i = 0; i < num_hidden_neurons_; i++) 
    {
        if (forward_layer2_(i) < 0)
            forward_layer2_(i) = 0.0;
    }

    forward_network_output_ = forward_W4 * forward_layer2_ + forward_b4;
} 

void SlavePandaController::computeTrainedModel()
{
    // while(ros::ok())
    // {
        if (is_init_)
        {
            int cur_idx = 0;
            if (ring_buffer_idx_ == 0)
                cur_idx = num_seq - 1;
            else
                cur_idx = ring_buffer_idx_ - 1;

            for (int seq = 0; seq < num_seq-1; seq++)
            {
                for (int input_feat = 0; input_feat < num_features*num_joint; input_feat++)
                {
                    int process_data_idx = cur_idx+seq+1;
                    if (process_data_idx >= num_seq)
                        process_data_idx -= num_seq;
                    condition_(seq*num_features*num_joint + input_feat) = ring_buffer_[process_data_idx*num_features*num_joint + input_feat];
                }
            }
            for (int input_feat = 0; input_feat < num_features*num_joint; input_feat++)
            {
                state_(input_feat) = ring_buffer_[cur_idx*num_features*num_joint + input_feat];
            }
            for (int i = 0; i < num_joint; i++)
                input_(i) = 2*(control_input_(i)+output_scaling(i))/(output_scaling(i)+output_scaling(i)) - 1; // ring_buffer_control_input_[cur_idx*num_joint + i];

            // computeForwardDynamicsModel();
            computeBackwardDynamicsModel();
            
            for (int i = 0; i < num_joint; i++)
                network_output_share_(i) = backward_network_output_(i);
        }
    // }
}


Eigen::Matrix7d SlavePandaController::getC(Eigen::Vector7d q, Eigen::Vector7d q_dot){
    double h = 2e-12;

    Eigen::VectorXd q_new;
    q_new.resize(dc_.num_dof_);
    
    Eigen::Matrix7d C, C1, C2;
    C.setZero();
    C1.setZero();
    C2.setZero();

    Eigen::MatrixXd A(dc_.num_dof_, dc_.num_dof_), A_new(dc_.num_dof_, dc_.num_dof_);
    Eigen::MatrixXd m[dc_.num_dof_];
    double b[dc_.num_dof_][dc_.num_dof_][dc_.num_dof_];

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        q_new = q;
        q_new(i) += h;

        A.setZero();
        A_new.setZero();

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q, A, true);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_new, A_new, true);

        m[i].resize(dc_.num_dof_, dc_.num_dof_);
        m[i] = (A_new - A) / h;
    }

    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int k = 0; k < dc_.num_dof_; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));


    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            C1(i, j) = b[i][j][j] * q_dot(j);

    for (int k = 0; k < dc_.num_dof_; k++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int i = 1 + j; i < dc_.num_dof_; i++)
            C2(k, j) += 2.0 * b[k][j][i] * q_dot(i);
    C = C1 + C2;

    return C;
}

// "Sliding Mode Momentum Observers for Estimation of External Torques and Joint Acceleration"
void SlavePandaController::computeSOSML()
{
    p_ = A_*q_dot_;
    p_tilde_ = p_hat_ - p_;

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        if (p_tilde_(i) > 0)
            p_tilde_sign_(i) = 1;
        else if (p_tilde_(i) == 0.0)
            p_tilde_sign_(i) = 0.0;
        else 
            p_tilde_sign_(i) = -1;
    }

    p_hat_ = p_hat_ + (control_input_ + C_.transpose()*q_dot_ - g_ - T1_*p_tilde_.cwiseAbs().cwiseSqrt().cwiseProduct(p_tilde_sign_) - T2_*p_tilde_ + sigma_) / hz_;
    sigma_ = sigma_ + (-S1_*p_tilde_sign_ - S2_*p_tilde_) / hz_;

    estimated_ext_torque_SOSML_ = -sigma_;
}

// "Interaction Force Estimation Using Extended State Observers: An Application to Impedance-Based Assistive and Rehabilitation Robotics"
void SlavePandaController::computeESO()
{
    x1_ = q_;
    x1_tilde_ = x1_ - x1_hat_;

    A_ESO_.setZero();
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, x1_hat_, A_ESO_, true);
    Eigen::Vector7d q_dot_zero;
    q_dot_zero.setZero();
    RigidBodyDynamics::NonlinearEffects(robot_, x1_hat_, q_dot_zero, g_ESO_);
    C_ESO_ = getC(x1_hat_, x2_hat_);

    x1_hat_ = x1_hat_ + (x2_hat_ + eta1_/epsilon_*x1_tilde_) / hz_;
    x2_hat_ = x2_hat_ + (A_ESO_.inverse()*(-C_ESO_*x2_hat_ - g_ESO_) + A_.inverse()*control_input_ + A_.inverse()*x3_hat_ + eta2_/pow(epsilon_,2)*x1_tilde_)/ hz_;
    x3_hat_ = x3_hat_ + (1/pow(epsilon_,3) * x1_tilde_) / hz_;

    estimated_ext_torque_ESO_ = -x3_hat_;
}

void SlavePandaController::computeHOFTO()
{
    x1_HOFTO_ = q_;
    x2_HOFTO_ = q_dot_;

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, x1_HOFTO_, A_HOFTO_, true);
    RigidBodyDynamics::NonlinearEffects(robot_, x1_HOFTO_, x2_HOFTO_, non_linear_HOFTO_);

    x1_z1_diff_ = x1_HOFTO_ - z1_;

    Eigen::Vector7d pow_m2, pow_m3, pow_m4, pow_m5; 
    for (int i = 0; i < dc_.num_dof_; i++)
    {
        pow_m2(i) = pow(x1_z1_diff_(i), m2_);
        pow_m3(i) = pow(x1_z1_diff_(i), m3_);
        pow_m4(i) = pow(x1_z1_diff_(i), m4_);
        pow_m5(i) = pow(x1_z1_diff_(i), m5_);
    }

    z1_ = z1_ + (z2_ + L1_*pow_m2)/hz_;
    z2_ = z2_ + (z3_ + A_HOFTO_.inverse()*control_input_ - A_HOFTO_.inverse()*non_linear_HOFTO_ + L2_*pow_m3)/hz_;
    z3_ = z3_ + (z4_ + L3_*pow_m4)/hz_;
    z4_ = z4_ + (L4_*pow_m5)/hz_;

    estimated_ext_torque_HOFTO_ = -A_HOFTO_ * z3_;
}