
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <thread>

#include "panda_controller/slave_panda_controller.h"
#include "panda_controller/master_panda_controller.h"
#include "panda_controller/mujoco_interface.h"

#define TorqueControl 1
#define PositionControl 0

#define Master true
#define Slave false

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_controller");
    ros::NodeHandle nh_master;
    ros::NodeHandle nh_slave;

    DataContainer dc_master;
    DataContainer dc_slave;

    int control_mode = PositionControl;

    MujocoInterface mujoco_interface_master(nh_master, dc_master, Master);
    MujocoInterface mujoco_interface_slave(nh_slave, dc_slave, Slave);

    std::thread thread[6];
    int num_thread = 0;


    MasterPandaController panda_controller_master(nh_master, dc_master, control_mode);
    thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface_master);
    thread[1] = std::thread(&MasterPandaController::compute, &panda_controller_master);
    thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface_master, control_mode);
    num_thread += 3;

    SlavePandaController panda_controller_slave(nh_slave, dc_slave, control_mode);
    thread[3] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface_slave);
    thread[4] = std::thread(&SlavePandaController::compute, &panda_controller_slave);
    thread[5] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface_slave, control_mode);
    num_thread += 3;
        

    for (int i = 0; i < num_thread; i++)
    {
        thread[i].join();
    }

    return 0;
}
