#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc, int control_mode) : dc_(dc), move_group_(PLANNING_GROUP)
{
    if (control_mode == 0)
        dc_.sim_mode_ = "position";
    else if (control_mode == 1)
        dc_.sim_mode_ = "torque";

    // RBDL
    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout<<"Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, false, false);

    // Moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();
    initMoveit();
    
    // Logging
    if (is_write_)
    {
        writeFile.open("/home/kim/ssd2/data.csv", std::ofstream::out | std::ofstream::app);
        writeFile << std::fixed << std::setprecision(8);
    }

    // Torch
    try {
        trained_model_ = torch::jit::load("/home/kim/ssd2/PandaResidual/model/traced_model.pt");
    }
    catch (const c10::Error& e) {
        std::cerr << "Error loading the model\n";
    }

    // Keyboard
    init_keyboard();
}

PandaController::~PandaController()
{

}

void PandaController::initMoveit()
{
    const robot_state::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group_.setMaxVelocityScalingFactor(5.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    setMoveitObstables();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

void PandaController::setMoveitObstables()
{
    // Create collision object
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    double square_len = 4.0;
    double hole_len = 0.22;
    // Box 1
    primitive.dimensions[0] = (square_len - hole_len)/2.0;
    primitive.dimensions[1] = square_len;
    primitive.dimensions[2] = 0.01;
    moveit_msgs::CollisionObject box1;
    box1.header.frame_id = move_group_.getPlanningFrame();
    box1.id = "box1";
    geometry_msgs::Pose box1_pose;
    box1_pose.position.x = (hole_len + primitive.dimensions[0])/2.0;
    box1_pose.position.y = 0.0;
    box1_pose.position.z = 0.095;
    box1.primitives.push_back(primitive);
    box1.primitive_poses.push_back(box1_pose);
    box1.operation = box1.ADD;
    collision_objects.push_back(box1);

    // Box 2
    moveit_msgs::CollisionObject box2;
    box2.header.frame_id = move_group_.getPlanningFrame();
    box2.id = "box2";
    geometry_msgs::Pose box2_pose;
    box2_pose.position.x = -(hole_len + primitive.dimensions[0])/2.0;
    box2_pose.position.y = 0.0;
    box2_pose.position.z = 0.095;
    box2.primitives.push_back(primitive);
    box2.primitive_poses.push_back(box2_pose);
    box2.operation = box2.ADD;
    collision_objects.push_back(box2);

    // Box 3
    primitive.dimensions[0] = hole_len;
    primitive.dimensions[1] = (square_len - hole_len)/2.0;;
    moveit_msgs::CollisionObject box3;
    box3.header.frame_id = move_group_.getPlanningFrame();
    box3.id = "box3";
    geometry_msgs::Pose box3_pose;
    box3_pose.position.x = 0.0;
    box3_pose.position.y = (primitive.dimensions[1] + hole_len) / 2.0;
    box3_pose.position.z = 0.095;
    box3.primitives.push_back(primitive);
    box3.primitive_poses.push_back(box3_pose);
    box3.operation = box3.ADD;
    collision_objects.push_back(box3);

    // Box 4
    moveit_msgs::CollisionObject box4;
    box4.header.frame_id = move_group_.getPlanningFrame();
    box4.id = "box4";
    geometry_msgs::Pose box4_pose;
    box4_pose.position.x = 0.0;
    box4_pose.position.y = -(primitive.dimensions[1] + hole_len) / 2.0;;
    box4_pose.position.z = 0.095;
    box4.primitives.push_back(primitive);
    box4.primitive_poses.push_back(box4_pose);
    box4.operation = box4.ADD;
    collision_objects.push_back(box4);

    planning_scene_interface_.addCollisionObjects(collision_objects);
}

void PandaController::generateRandTraj()
{
    if (!next_traj_prepared_)
    {
        moveit::core::RobotState start_state = *(move_group_.getCurrentState());

        for (int i = 0; i < dc_.num_dof_; i++)
        {
            q_init_plan_[i] = q_target_plan_[i];
            q_dot_plan_[i] = 0.0;
        }

        start_state.setJointGroupPositions(PLANNING_GROUP, q_init_plan_);
        start_state.setJointGroupVelocities(PLANNING_GROUP, q_dot_plan_);
        move_group_.setStartState(start_state);
        
        std::random_device rand_device;
        std::default_random_engine rand_seed;
        std::uniform_real_distribution<double> angles[dc_.num_dof_];
        rand_seed.seed(rand_device());

        do
        {
            double safe_range = 0.0;
            for (size_t i = 0; i < dc_.num_dof_; i++)
            {
                safe_range = ((q_limit_u_[i] - q_limit_l_[i]) * 0.1);
                angles[i] = std::uniform_real_distribution<double>((q_limit_l_[i] + safe_range), (q_limit_u_[i] - safe_range));
            }

            for (int i = 0; i < dc_.num_dof_; i++) 
                q_target_plan_[i] = ((angles[i])(rand_seed));

            move_group_.setJointValueTarget(q_target_plan_);
        } 
        while (move_group_.plan(random_plan_next_) != moveit::planning_interface::MoveItErrorCode::SUCCESS);

        next_traj_prepared_ = true;
        std::cout<<"Trajectory prepared"<<std::endl;
    }
}

void PandaController::generateRandTrajThread()
{
    ros::Rate r(10);
    while(ros::ok())
    {
        generateRandTraj();
        if (!init_traj_prepared_)
            init_traj_prepared_ = true;
        r.sleep();
    }
}

void PandaController::compute()
{
    ros::Rate r(hz_);
    while(ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (!is_init_)
            {
                // Robot State
                q_.resize(dc_.num_dof_);
                q_.setZero();
                q_dot_.resize(dc_.num_dof_);
                q_dot_.setZero();
                effort_.resize(dc_.num_dof_);
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
                q_ddot_desired_.resize(dc_.num_dof_);
                q_ddot_desired_.setZero();
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

                non_linear_.resize(dc_.num_dof_);
                non_linear_.setZero();
                A_.resize(dc_.num_dof_, dc_.num_dof_);
                A_.setZero();
                C_.resize(dc_.num_dof_, dc_.num_dof_);
                C_.setZero();
                Lambda_.resize(6,6);
                Lambda_.setZero();

                // For Moveit
                q_limit_l_.resize(dc_.num_dof_);
                q_limit_u_.resize(dc_.num_dof_);
                q_limit_l_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0873, -2.8973;
                q_limit_u_ <<  2.8973,  1.7628,  2.8973,  0.0698,  2.8973,  2.1127,  2.8973;

                q_target_plan_.resize(dc_.num_dof_);
                q_init_plan_.resize(dc_.num_dof_);
                q_dot_plan_.resize(dc_.num_dof_);
                std::fill(q_target_plan_.begin(), q_target_plan_.end(), 0);
                std::fill(q_init_plan_.begin(), q_init_plan_.end(), 0);
                std::fill(q_dot_plan_.begin(), q_dot_plan_.end(), 0);

                // Torch
                std::fill(ring_buffer_, ring_buffer_+num_seq*num_features*num_joint, 0);
                estimated_ext_torque_.resize(dc_.num_dof_);
                estimated_ext_torque_.setZero();
                estimated_ext_torque_filtered_.resize(dc_.num_dof_);
                estimated_ext_torque_filtered_.setZero();
                measured_ext_torque_.resize(dc_.num_dof_);
                measured_ext_torque_.setZero();

                estimated_ext_force_.resize(6);
                estimated_ext_force_.setZero();
                estimated_ext_force_pre_.resize(6);
                estimated_ext_force_pre_.setZero();
                estimated_ext_force_init_.resize(6);
                estimated_ext_force_init_.setZero();
                measured_ext_force_.resize(6);
                measured_ext_force_.setZero();

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

                T1_.diagonal() << 2*sqrt(80), 2*sqrt(80), 2*sqrt(80), 2*sqrt(80), 2*sqrt(80), 2*sqrt(80), 2*sqrt(80);
                T2_.diagonal() << 80, 80, 80, 80, 80, 80, 80;
                S1_.diagonal() << 80, 80, 80, 80, 80, 80, 80;
                S2_.diagonal() << 1600, 1600, 1600, 1600, 1600, 1600, 1600;

                init_time_ = ros::Time::now().toSec();

                is_init_ = true;
            }

            if (is_init_ && init_traj_prepared_)
            {
                cur_time_= ros::Time::now().toSec() - init_time_;

                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                q_ = dc_.q_;
                q_dot_ = dc_.q_dot_;
                effort_ = dc_.effort_;
                m_dc_.unlock();

                updateKinematicsDynamics();
                computeSOSML();
                computeControlInput();  
                writeBuffer();
                if (is_write_)
                {
                    logData();
                }
                
                pre_time_ = cur_time_;
            }

            if (_kbhit()) {
                int ch = _getch();
                _putch(ch);
                mode_ = ch;

                mode_init_time_ = ros::Time::now().toSec() - init_time_;
                q_mode_init_ = q_;
                q_dot_mode_init_ = q_dot_;
                x_mode_init_ = x_;

                m_ext_.lock();
                estimated_ext_force_init_ = estimated_ext_force_;
                m_ext_.unlock();

                std::cout << "Mode Changed to: ";   // i: 105, r: 114, m: 109, s: 115, f:102, h: 104
                switch(mode_)
                {
                    case(104):
                        std::cout << "Home Pose" << std::endl;
                        break;
                    case(105):
                        std::cout<< "Init Pose" << std::endl;
                        break;
                    case(114):
                        std::cout<< "Random Trajectory" << std::endl;
                        break;
                    case(102):
                        std::cout << "Force Control" << std::endl;
                        break;
                }
            }
        ros::spinOnce();
        r.sleep();
        }
    }
    close_keyboard();
}

void PandaController::updateKinematicsDynamics()
{
    static const int BODY_ID = robot_.GetBodyId("panda_link8");

    m_rbdl_.lock();
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

    getC();
    m_rbdl_.unlock();

    Lambda_ = (j_ * A_.inverse() * j_.transpose()).inverse();
}

void PandaController::computeControlInput()
{
    if(mode_ == MODE_RANDOM)
    {
        if (cur_time_ >= traj_init_time_ + traj_duration_)
        {   
            random_plan_ = random_plan_next_;
            cur_waypoint_ = 0;
            traj_init_time_ = cur_time_;//ros::Time::now().toSec();
            total_waypoints_ = random_plan_.trajectory_.joint_trajectory.points.size();
            traj_duration_ = random_plan_.trajectory_.joint_trajectory.points[total_waypoints_-1].time_from_start.toSec();

            next_traj_prepared_ = false; 
            std::vector<double> way = random_plan_.trajectory_.joint_trajectory.points[total_waypoints_-1].positions;
            std::cout<<"New Trajectory!"<< std::endl;
            std::cout<<"Total Waypoint: "<< total_waypoints_ << std::endl;
            std::cout << "Start Pose: " << q_(0) << " " << q_(1) << " " << q_(2) << " " << q_(3) << " " << q_(4) << " " << q_(5) << " " << q_(6) << std::endl;
            std::cout << "Target Pose: " << way[0] << " " << way[1] << " " << way[2] << " " << way[3] << " " << way[4] << " " << way[5] << " " << way[6] << std::endl;
            std::cout<<"Trajetory Duration: " << traj_duration_ << std::endl << std::endl;
        }

        if (cur_time_ >= traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].time_from_start.toSec())
        {
            if (cur_waypoint_ < total_waypoints_-2)
                cur_waypoint_++;
        }
        
        double way_point_start_time = traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].time_from_start.toSec();
        double way_point_end_time = traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].time_from_start.toSec();

        std::vector<Eigen::Vector3d> traj;
        traj.resize(dc_.num_dof_);

        for (int i = 0; i < dc_.num_dof_; i++)
        {
            double init_q = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].positions[i];
            double init_q_dot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].velocities[i];
            double init_q_ddot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_].accelerations[i];
            double target_q = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].positions[i];
            double target_q_dot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].velocities[i];
            double target_q_ddot = random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].accelerations[i];

            traj[i] = quintic_spline(cur_time_, way_point_start_time, way_point_end_time, init_q, init_q_dot, init_q_ddot, target_q, target_q_dot, target_q_ddot);

            q_desired_(i) = traj[i](0);
            q_dot_desired_(i) = traj[i](1);
            q_ddot_desired_(i) = traj[i](2);
        }

        control_input_ = A_*(q_ddot_desired_ + kv*(q_dot_desired_ - q_dot_) + kp * (q_desired_ - q_))+ non_linear_;
    }

    else if (mode_ == MODE_HOME)
    {
        Eigen::VectorXd q_target;
        q_target.resize(dc_.num_dof_);
        q_target << 0.0, -3.14/6, 0.0, -3.14/6, 0.0, 0.0, 0.0;

        for(int i = 0; i < dc_.num_dof_; i++)
        {
            Eigen::Vector3d traj_desired;
            traj_desired = quintic_spline(cur_time_, mode_init_time_, mode_init_time_+5.0, q_mode_init_(i), q_dot_mode_init_(i), 0.0, q_target(i), 0.0, 0.0);
            q_desired_(i) = traj_desired(0);
            q_dot_desired_(i) = traj_desired(1);
            q_ddot_desired_(i) = traj_desired(2);
        }

        control_input_ = A_*(q_ddot_desired_ + kv*(q_dot_desired_ - q_dot_) + kp * (q_desired_ - q_))+ non_linear_;
    }

    else if (mode_ == MODE_INIT)
    {
        x_target_.translation() << 0.3, 0.0, 0.8;
        x_target_.linear() << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;

        double traj_duration = 5.0;

        for (int i = 0; i < 3; i++)
        {
            x_desired_.translation()(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
            x_dot_desired_(i) = cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
        }
        
        x_desired_.linear() = rotationCubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 
        x_dot_desired_.segment(3,3) = rotationCubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 

        Eigen::VectorXd x_error;
        x_error.resize(6);
        x_error.setZero();
        Eigen::VectorXd x_dot_error;
        x_dot_error.resize(6);
        x_dot_error.setZero();

        x_error.segment(0,3) = x_desired_.translation() - x_.translation();
        x_error.segment(3,3) = -getPhi(x_.linear(), x_desired_.linear());
        x_dot_error.segment(0,3)= x_dot_desired_.segment(0,3) - x_dot_.segment(0,3);
        x_dot_error.segment(3,3)= x_dot_desired_.segment(3,3) - x_dot_.segment(3,3);

        control_input_ = j_.transpose()*Lambda_*(kp_task_*x_error +kv_task_*x_dot_error) + non_linear_;
    }

    else if (mode_ == MODE_FORCE)
    {
        double traj_duration = 5.0;

        Eigen::VectorXd f_star;
        f_star.resize(6);

        // Position control y, z
        x_target_.translation() << 0.3, 0.0, 0.8;
        x_target_.linear() << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;

        for (int i = 0; i < 3; i++)
        {
            x_desired_.translation()(i) = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
            x_dot_desired_(i) = cubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
        }
        
        x_desired_.linear() = rotationCubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 
        x_dot_desired_.segment(3,3) = rotationCubicDot(cur_time_, mode_init_time_, mode_init_time_+traj_duration, x_mode_init_.linear(), x_target_.linear()); 

        Eigen::VectorXd x_error;
        x_error.resize(6);
        x_error.setZero();
        Eigen::VectorXd x_dot_error;
        x_dot_error.resize(6);
        x_dot_error.setZero();

        x_error.segment(0,3) = x_desired_.translation() - x_.translation();
        x_error.segment(3,3) = -getPhi(x_.linear(), x_desired_.linear());
        x_dot_error.segment(0,3)= x_dot_desired_.segment(0,3) - x_dot_.segment(0,3);
        x_dot_error.segment(3,3)= x_dot_desired_.segment(3,3) - x_dot_.segment(3,3);

        f_star = kp_task_*x_error +kv_task_*x_dot_error;

        // Force control x
        double f_d_x = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, estimated_ext_force_init_(0), 20.0, 0.0, 0.0);
        double k_p_force = 0.05;
        double k_v_force = 0.001;
        m_ext_.lock();
        f_star(0) = k_p_force*(f_d_x - estimated_ext_force_(0)) + k_v_force*(estimated_ext_force_(0) - estimated_ext_force_pre_(0))/hz_;
        m_ext_.unlock();
        estimated_ext_force_pre_ = estimated_ext_force_;

        // Eigen::VectorXd F_d;
        // F_d.resize(6);
        // F_d.setZero();
        // F_d(0) = f_d_x;

        // control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;

        f_star(0) = 0.0;
        f_I_ = f_I_ + 1.0 * (f_d_x - measured_ext_force_(0)) / hz_;

        Eigen::VectorXd F_d;
        F_d.resize(6);
        F_d.setZero();
        F_d(0) = f_d_x + f_I_;
        
        control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;
    }

    else
    {
        control_input_ = non_linear_;
    }
    
    m_ci_.lock();
    dc_.control_input_ = control_input_;
    m_ci_.unlock();
}

void PandaController::writeBuffer()
{
    if (int(cur_time_*100) != int(pre_time_*100))
    {
        m_buffer_.lock();
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i] = 2*(q_(i)-min_theta_)/(max_theta_-min_theta_) - 1;
            ring_buffer_[ring_buffer_idx_*num_features*num_joint + num_features*i + 1] = 2*(q_dot_(i)-min_theta_dot_)/(max_theta_dot_-min_theta_dot_) - 1;
        }

        ring_buffer_idx_++;
        if (ring_buffer_idx_ == num_seq)
            ring_buffer_idx_ = 0;

        m_buffer_.unlock();
    }
}

void PandaController::logData()
{
    if (int(cur_time_*100) != int(pre_time_*100))
    {
        writeFile << cur_time_ << "\t";
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << q_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << q_dot_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << q_desired_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << q_dot_desired_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << q_ddot_desired_(i) << "\t";
        // }
        // for (int i = 0; i < dc_.num_dof_; i++)
        // {
        //     writeFile << control_input_(i) << "\t";
        // }

        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << estimated_ext_torque_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << measured_ext_torque_(i) << "\t";
        }
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << -sigma_(i) << "\t";
        }
        writeFile << "\n";
    }
}

void PandaController::computeTrainedModel()
{
    while(ros::ok())
    {
        if (is_init_)
        {
            cur_time_inference_= ros::Time::now().toSec() - init_time_;

            m_buffer_.lock();
            for (int seq = 0; seq < num_seq; seq++)
            {
                for (int input_feat = 0; input_feat < num_features*num_joint; input_feat++)
                {
                    int process_data_idx = ring_buffer_idx_+seq;
                    if (process_data_idx >= num_seq)
                        process_data_idx -= num_seq;
                    input_tensor_[0][seq][input_feat] = ring_buffer_[process_data_idx*num_features*num_joint + input_feat];
                }
            }
            m_buffer_.unlock();
                
            // Create a vector of inputs.
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(input_tensor_);

            // Execute the model and turn its output into a tensor.
            at::Tensor output = trained_model_.forward(inputs).toTensor();
            // double* temp_arr = output.data_ptr<double>();

            Eigen::Matrix<double, 7, 1> ext;
            ext.setZero();
            ext = (A_*dc_.effort_ + non_linear_) - control_input_;

            for (int i= 0; i < dc_.num_dof_; i++)
            {
                estimated_ext_torque_(i) = control_input_[i] - output.data()[0][i].item<double>()*100;

                if (i < 4)
                {
                    measured_ext_torque_(i) = -(ext(i) + dc_.q_dot_(i) * 100.0);
                }
                else
                {
                    measured_ext_torque_(i) = -(ext(i) + dc_.q_dot_(i) * 10.0);
                }

                estimated_ext_torque_filtered_(i) = lowPassFilter(estimated_ext_torque_(i), estimated_ext_torque_filtered_(i), 1/hz_, 20.0);
            }
            
            m_rbdl_.lock();
            m_ext_.lock();
            estimated_ext_force_ = (j_.transpose()*((j_*j_.transpose()).inverse())).transpose()*estimated_ext_torque_filtered_;
            measured_ext_force_ = (j_.transpose()*((j_*j_.transpose()).inverse())).transpose()*measured_ext_torque_;
            m_ext_.unlock();
            m_rbdl_.unlock();
            
            if (int(cur_time_inference_*10) != int(pre_time_inference_*10))
            {
                    std::cout <<"Estimated Force: " << estimated_ext_force_(0) <<"\t"<< estimated_ext_force_(1) <<"\t"<< estimated_ext_force_(2) <<std::endl;

                    // Eigen::Vector3d measured_force;
                    // measured_force = x_.linear() * dc_.force_;

                    // std::cout <<"FT Measured Force: " << measured_force(0) <<"\t"<< measured_force(1) <<"\t"<< measured_force(2) <<std::endl;

                    std::cout <<"Measured Force: " << measured_ext_force_(0) <<"\t"<< measured_ext_force_(1) <<"\t"<< measured_ext_force_(2) <<std::endl;
                    
                    // std::cout <<"LSTM Ext: " << measured_ext_torque_(0) <<"\t"<< measured_ext_torque_(1) <<"\t"<< measured_ext_torque_(2) <<std::endl;
                    // std::cout <<"SOSML Ext: " << sigma_(0) <<"\t"<< sigma_(1) <<"\t"<< sigma_(2) <<std::endl;
            }

            pre_time_inference_ = cur_time_inference_;
        }
    }
}


void PandaController::getC(){
    double h = 2e-12;

    Eigen::VectorXd q_new;
    q_new.resize(dc_.num_dof_);
    
    Eigen::MatrixXd C1(dc_.num_dof_, dc_.num_dof_);
    C1.setZero();
    Eigen::MatrixXd C2(dc_.num_dof_, dc_.num_dof_);
    C2.setZero();

    Eigen::MatrixXd A(dc_.num_dof_, dc_.num_dof_), A_new(dc_.num_dof_, dc_.num_dof_);
    Eigen::MatrixXd m[dc_.num_dof_];
    double b[dc_.num_dof_][dc_.num_dof_][dc_.num_dof_];

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        q_new = q_;
        q_new(i) += h;

        A.setZero();
        A_new.setZero();

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, A_, true);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_new, A_new, true);

        m[i].resize(dc_.num_dof_, dc_.num_dof_);
        m[i] = (A_new - A_) / h;
    }

    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int k = 0; k < dc_.num_dof_; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));


    for (int i = 0; i < dc_.num_dof_; i++)
        for (int j = 0; j < dc_.num_dof_; j++)
            C1(i, j) = b[i][j][j] * q_dot_(j);

    for (int k = 0; k < dc_.num_dof_; k++)
        for (int j = 0; j < dc_.num_dof_; j++)
            for (int i = 1 + j; i < dc_.num_dof_; i++)
            C2(k, j) += 2.0 * b[k][j][i] * q_dot_(i);
    C_ = C1 + C2;
}

void PandaController::computeSOSML()
{
    p_ = A_*q_dot_;
    p_tilde_ = p_hat_ - p_;

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        if (p_tilde_(i) >= 0)
            p_tilde_sign_(i) = 1;
        else
            p_tilde_sign_(i) = -1;
    }

    p_hat_ = p_hat_ + (control_input_ + C_.transpose()*q_dot_ - g_ - T1_*p_tilde_.cwiseAbs().cwiseSqrt().cwiseProduct(p_tilde_sign_) - T2_*p_tilde_ + sigma_) / hz_;
    sigma_ = sigma_ + (-S1_*p_tilde_sign_ - S2_*p_tilde_) / hz_;
}