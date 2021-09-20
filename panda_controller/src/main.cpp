
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

    int control_mode = PositionControl;

    MujocoInterface mujoco_interface(nh, dc);
    PandaController panda_controller(nh, dc, control_mode);

    std::thread thread[4];
    thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
    thread[1] = std::thread(&PandaController::compute, &panda_controller);
    // thread[2] = std::thread(&PandaController::computeTrainedModel, &panda_controller);
    thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, control_mode);
    // thread[3] = std::thread(&PandaController::generateRandTrajThread, &panda_controller);

    for (int i = 0; i < 3; i++)
    {
        thread[i].join();
    }

    return 0;
}
