
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <thread>

#include "panda_controller/panda_controller.h"
#include "panda_controller/mujoco_interface.h"

#define TorqueControl 1
#define PositionControl 0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_controller");
    ros::NodeHandle nh;
    DataContainer dc;

    MujocoInterface mujoco_interface(nh, dc);
    PandaController panda_controller(nh, dc);

    std::thread thread[5];
    thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
    thread[1] = std::thread(&PandaController::compute, &panda_controller);
    thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, TorqueControl);

    for (int i = 0; i < 3; i++)
    {
        thread[i].join();
    }

    return 0;
}
