
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <thread>

#include "panda_controller/slave_panda_controller.h"
#include "panda_controller/master_panda_controller.h"
#include "panda_controller/mujoco_interface.h"

#define TorqueControl 1
#define PositionControl 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_controller");
    ros::NodeHandle nh;
    DataContainer dc;

    int control_mode = PositionControl;

    MujocoInterface mujoco_interface(nh, dc);

    std::thread thread[5];
    int num_thread = 0;

    std::cout << "Robot Type: " << ros::this_node::getNamespace() << std::endl;
    if (ros::this_node::getNamespace() == "/master")
    {
        MasterPandaController panda_controller(nh, dc, control_mode);
        thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
        thread[1] = std::thread(&MasterPandaController::compute, &panda_controller);
        thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, control_mode);
        num_thread = 3;
        std::cout << "Master Ready" << std::endl;
    }
    else if (ros::this_node::getNamespace() == "/slave")
    {
        SlavePandaController panda_controller(nh, dc, control_mode);
        thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
        thread[1] = std::thread(&SlavePandaController::compute, &panda_controller);
        // thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, control_mode);
        num_thread = 2;
        std::cout<< "Slave Ready" << std::endl;
    }
    else
    {
        std::cout << "No Type" << std::endl;
    }

    for (int i = 0; i < num_thread; i++)
    {
        thread[i].join();
    }

    return 0;
}
