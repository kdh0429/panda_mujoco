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
        writeFile.open("/home/kim/ssd2/residualData.csv", std::ofstream::out | std::ofstream::app);
        writeFile << std::fixed << std::setprecision(8);
    }

    // Torch
    loadNetwork();

    // Ros
    resi_publisher_ = nh.advertise<std_msgs::Float32MultiArray>("/panda/residual", 1000);

    // Keyboard
    init_keyboard();

    init_traj_prepared_ = true;
}

PandaController::~PandaController()
{

}

void PandaController::initMoveit()
{
    const robot_state::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group_.setMaxVelocityScalingFactor(1.0);

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
                control_input_filtered_.setZero();

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

                // ROS OCSVM
                resi_msg_.data.resize(num_joint*num_seq);

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

                // printData();            

                publishResidual();    

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

                estimated_ext_force_init_ = estimated_ext_force_;

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
                    case(115):
                        std::cout << "Stop" << std::endl;
                        break;
                }
            }
        ros::spinOnce();
        // r.sleep();
        }
    }
    close_keyboard();
}

void PandaController::updateKinematicsDynamics()
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

void PandaController::computeControlInput()
{
    if(mode_ == MODE_RANDOM)
    {
        if (cur_time_ >= traj_init_time_ + traj_duration_ + 1.0)
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
        else if (cur_time_ >= traj_init_time_ + traj_duration_)
        {
            // Rest
        }
        else if (cur_time_ >= traj_init_time_ + random_plan_.trajectory_.joint_trajectory.points[cur_waypoint_+1].time_from_start.toSec())
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
        f_d_x_ = cubic(cur_time_, mode_init_time_, mode_init_time_+traj_duration, estimated_ext_force_init_(0), 40.0, 0.0, 0.0);
        double k_p_force = 0.05;
        double k_v_force = 0.001;
        
        f_star(0) = k_p_force*(f_d_x_ - estimated_ext_force_(0)) + k_v_force*(estimated_ext_force_(0) - estimated_ext_force_pre_(0))/hz_;
        
        estimated_ext_force_pre_ = estimated_ext_force_;

        // Eigen::VectorXd F_d;
        // F_d.resize(6);
        // F_d.setZero();
        // F_d(0) = f_d_x_;

        // control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;

        f_star(0) = 0.0;
        f_I_ = f_I_ + 1.0 * (f_d_x_ - estimated_ext_force_(0)) / hz_;
        

        Eigen::VectorXd F_d;
        F_d.resize(6);
        F_d.setZero();
        F_d(0) = f_d_x_ + f_I_;
        
        control_input_ = j_.transpose()*(Lambda_*f_star + F_d) + non_linear_;
    }
    else if (mode_ == MODE_STOP)
    {
        control_input_ = non_linear_;
    }
    else
    {
        control_input_ = non_linear_;
    }

    for (int i = 0; i < dc_.num_dof_; i++)
    {
        control_input_filtered_(i) = lowPassFilter(control_input_(i), control_input_filtered_(i), 1/hz_, 20);
    }
    
    dc_.control_input_ = control_input_; 
}

void PandaController::writeBuffer()
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

        for (int i = 0; i < dc_.num_dof_; i++)
        {
            resi_buffer_[resi_buffer_idx_*num_joint + i] = estimated_ext_torque_LSTM_(i);
        }
        
        resi_buffer_idx_++;
        if (resi_buffer_idx_ == num_seq)
            resi_buffer_idx_ = 0;
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
        //     writeFile << control_input_filtered_(i) << "\t";
        // }

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
        
        for (int i = 0; i < dc_.num_dof_; i++)
        {
            writeFile << estimated_ext_torque_LSTM_(i) << "\t";
        }
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

void PandaController::computeExtForce()
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

void PandaController::publishResidual()
{
    int cur_idx = 0;
    if (resi_buffer_idx_ == 0)
        cur_idx = num_seq - 1;
    else
        cur_idx = resi_buffer_idx_ - 1;

    for (int seq = 0; seq < num_seq; seq++)
    {
        for (int i=0; i < num_joint; i++)
        {
            int process_data_idx = cur_idx+seq+1;
            if (process_data_idx >= num_seq)
                process_data_idx -= num_seq;
           resi_msg_.data[num_joint*seq + i] = resi_buffer_[process_data_idx*num_joint + i];
        }
    }
    resi_publisher_.publish(resi_msg_);
}

void PandaController::printData()
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

void PandaController::loadNetwork()
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

void PandaController::computeBackwardDynamicsModel()
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

void PandaController::computeForwardDynamicsModel()
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

void PandaController::computeTrainedModel()
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


Eigen::Matrix7d PandaController::getC(Eigen::Vector7d q, Eigen::Vector7d q_dot){
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
void PandaController::computeSOSML()
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
void PandaController::computeESO()
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

void PandaController::computeHOFTO()
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