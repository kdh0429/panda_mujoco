#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc) : dc_(dc), move_group_(PLANNING_GROUP)
{
    ros::console::shutdown();

    dc_.sim_mode_ = "torque";
    
    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm.urdf";
    std::cout<<"Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &robot_, false, false);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    initMoveit();
}

PandaController::~PandaController()
{

}

void PandaController::initMoveit()
{
    const robot_state::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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
    std::vector<double> q_target;
    std::vector<double> q; 
    std::vector<double> q_dot; 
    q_target.resize(dc_.num_dof_-2);
    q.resize(dc_.num_dof_-2);
    q_dot.resize(dc_.num_dof_-2);

    moveit::core::RobotState start_state = *(move_group_.getCurrentState());
    for (int i = 0; i < dc_.num_dof_-2; i++)
    {
        q[i] = q_[i];
        q_dot[i] = q_dot_[i];
    }


    start_state.setJointGroupPositions(PLANNING_GROUP, q);
    start_state.setJointGroupVelocities(PLANNING_GROUP, q_dot);
    move_group_.setStartState(start_state);
    
    std::random_device rd;
    std::default_random_engine rand_seed;
    std::uniform_real_distribution<double> angles[dc_.num_dof_-2];
    rand_seed.seed(rd());

    do
    {
        double safe_range = 0.0;
        for (size_t i = 0; i < dc_.num_dof_-2; i++)
        {
            safe_range = ((q_limit_u_[i] - q_limit_l_[i]) * 0.1);
            angles[i] = std::uniform_real_distribution<double>((q_limit_l_[i] + safe_range), (q_limit_u_[i] - safe_range));
        }

        for (int i = 0; i < dc_.num_dof_-2; i++) 
            q_target[i] = ((angles[i])(rand_seed));

        move_group_.setJointValueTarget(q_target);
    } 
    while (move_group_.plan(random_plan_) != moveit::planning_interface::MoveItErrorCode::SUCCESS);

    std::cout<<"Planning Done" << std::endl;
    int waypoints = random_plan_.trajectory_.joint_trajectory.points.size();
    int waypoint = 0;
    double start_time = random_plan_.trajectory_.joint_trajectory.points[0].time_from_start.toSec();
    double end_time = random_plan_.trajectory_.joint_trajectory.points[waypoints - 1].time_from_start.toSec();
    std::cout<<"Start Time: "<< start_time << std::endl;
    std::cout<<"End Time: " << end_time << std::endl;
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
                // Robot State
                q_.resize(dc_.num_dof_);
                q_.setZero();
                q_dot_.resize(dc_.num_dof_);
                q_dot_.setZero();
                effort_.resize(dc_.num_dof_);
                effort_.setZero();

                // Control
                qddot_desired_.resize(dc_.num_dof_);
                qddot_desired_.setZero();
                qdot_desired_.resize(dc_.num_dof_);
                qdot_desired_.setZero();
                q_desired_.resize(dc_.num_dof_);
                q_desired_.setZero();

                kp = 1500;
                kv = 10;

                control_input_.resize(dc_.num_dof_);
                control_input_.setZero();

                non_linear_.resize(dc_.num_dof_);
                non_linear_.setZero();
                A_.resize(dc_.num_dof_, dc_.num_dof_);
                A_.setZero();

                // Joint Limits for Moveit
                q_limit_l_.resize(dc_.num_dof_-2);
                q_limit_u_.resize(dc_.num_dof_-2);
                q_limit_l_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0873, -2.8973;
                q_limit_u_ <<  2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.8223,  2.8973;
                
                traj_init_time_ = ros::Time::now().toSec();

                is_init_ = true;

                generateRandTraj();
            }

            if (is_init_)
            {
                m_dc_.lock();
                sim_time_ = dc_.sim_time_;
                q_ = dc_.q_;
                q_dot_ = dc_.q_dot_;
                effort_ = dc_.effort_;
                m_dc_.unlock();

                updateKinematicsDynamics();
                computeControlInput();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

void PandaController::updateKinematicsDynamics()
{
    RigidBodyDynamics::NonlinearEffects(robot_, q_, q_dot_, non_linear_);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(robot_, q_, A_, true);
}

void PandaController::computeControlInput()
{
    std::vector<Eigen::Vector3d> cmd;
    cmd.resize(dc_.num_dof_);

    double cur_time = ros::Time::now().toSec();
    for (int i = 0; i < dc_.num_dof_; i++)
    {
        cmd[i] = quintic_spline(cur_time, traj_init_time_, traj_init_time_+20.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0);
        q_desired_[i] = cmd[i](0);
        qdot_desired_[i] = cmd[i](1);
        qddot_desired_[i] = cmd[i](2);
    }
    control_input_ = A_*(qddot_desired_ + kv*(qdot_desired_ - q_dot_) + kp * (q_desired_ - q_))+ non_linear_;

    m_ci_.lock();
    dc_.control_input_ = control_input_;
    m_ci_.unlock();
}


Eigen::Vector3d PandaController::quintic_spline(
    double time,       // Current time
    double time_0,     // Start time
    double time_f,     // End time
    double x_0,        // Start state
    double x_dot_0,    // Start state dot
    double x_ddot_0,   // Start state ddot
    double x_f,        // End state
    double x_dot_f,    // End state
    double x_ddot_f    // End state ddot
)
{
    double a1, a2, a3, a4, a5, a6;
    double time_s;

    Eigen::Vector3d result;

    if (time < time_0)
    {
        result << x_0, x_dot_0, x_ddot_0;
        return result;
    }
    else if (time > time_f)
    {
        result << x_f, x_dot_f, x_ddot_f;
        return result;
    }

    time_s = time_f - time_0;
    a1 = x_0; a2 = x_dot_0; a3 = (x_ddot_0 / 2.0);

    Eigen::Matrix3d Temp;
    Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        (3.0 * pow(time_s, 2)), (4.0 * pow(time_s, 3)), (5.0 * pow(time_s, 4)),
        (6.0 * time_s), (12.0 * pow(time_s, 2)), (20.0 * pow(time_s, 3));

    Eigen::Vector3d R_temp;
    R_temp << (x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0),
        (x_dot_f - x_dot_0 - x_ddot_0 * time_s),
        (x_ddot_f - x_ddot_0);

    Eigen::Vector3d RES;

    RES = (Temp.inverse() * R_temp);
    a4 = RES(0); a5 = RES(1); a6 = RES(2);

    double time_fs = (time - time_0);
    double position = (a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5));
    double velocity = (a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4));
    double acceleration = (2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3));
    result << position, velocity, acceleration;

    return result;
}