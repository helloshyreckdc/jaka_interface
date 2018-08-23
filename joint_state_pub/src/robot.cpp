#include "robot_client.h"
#include "robot.h"

#include <iostream>
#include "unistd.h"

Robot::Robot(std::string address) : client(address)
{}

Robot::~Robot()
{}

int Robot::start()
{
    std::string username("JAKA");
    std::string password("JAKA");

    std::string reply = client.LogIn(username, password);
    std::cout << "Login: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    if (!reply.empty()) return -1;

    // reply = client.StateChange(STATE_ESTOPREST);
    // printf(" statechange 1!\n");
    // std::cout << "ESTOPREST: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    // if (!reply.empty()) return -2;

    // sleep(5);

    // reply = client.StateChange(STATE_PWRON);
    // printf(" statechange 2!\n");

    // std::cout << "PWRON: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    // if (!reply.empty()) return -3;

    // sleep(10);

    return 0;
}

int Robot::stop()
{
    std::string username("JAKA");
    std::string password("JAKA");
    std::string reply;

    reply = client.StateChange(STATE_PWROFF);
    printf(" statechange 3!\n");
    std::cout << "PWROFF: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    if (!reply.empty()) return -1;

    reply = client.StateChange(STATE_ESTOP);
    printf(" statechange 4!\n");
    std::cout << "ESTOP: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    if (!reply.empty()) return -2;

    reply = client.LogOut(username, password);
    std::cout << "Logout: " << (reply.empty() ? "\t\tSuccess":reply) << std::endl;
    if (!reply.empty()) return -3;

    return 0;
}


int Robot::go(Eigen::Matrix4f targetPosition, double velocity)
{
    return 0;
}

int Robot::asyncGo(Eigen::Matrix4f targetPosition, double velocity)
{
    Eigen::Matrix3f rotation = targetPosition.topLeftCorner(3,3);
    Eigen::Vector3f rpy = rotation.eulerAngles(0,1,2);
    Eigen::Vector3f trans = targetPosition.topRightCorner(3,1);
    std::vector<float> currjnt;
    std::vector<float> cart;
    std::vector<float> world;
    std::vector<float> joints;

    for (int i=0;i<3;i++) world.push_back(trans(i,0));
    for (int i=0;i<3;i++) world.push_back(rpy(i,0)*180/M_PI);

    //client.GetRobotPose(currjnt,cart);
    client.KineInverse(world,currjnt,joints);
    this->jointMove(joints,velocity);

    return 0;//jointMove();
}

int Robot::jointMove(std::vector<float> pos_vector, float velocity)
{
    int reply;
    reply = client.MovJ(pos_vector, velocity);
    std::cout << std::endl << "pos_vector:";
    for (int i=0;i<6;i++) std::cout << " " << pos_vector.at(i);
    std::cout << std::endl;
    std::cout << "MovJ: " << (!reply?"Success":"Failed") << std::endl;
    if (!reply)
        return 0;
    else
        return -1;
}

int Robot::ServoJEnable(bool ena)
{
    int reply;
    reply=client.ServojopEnable( ena);
    return reply;
}


int Robot::servoJMove(std::vector<float> pos_vector, float t=8)
{
    int reply;
    reply = client.ServoJ(pos_vector,t);
    std::cout << std::endl << "pos_vector:";
    for (int i=0;i<6;i++) 
    // std::cout << " " << pos_vector.at(i);
    // std::cout << std::endl;
    std::cout << "srevoJ: " << (!reply?"Success":"Failed") << std::endl;
    if (!reply)
        return 0;
    else
        return -1;
}

int Robot::goHome()
{
    return 0;
}

// int Robot::getPose(Eigen::Isometry3f &pose)
// int Robot::getPose(std::vector<float> &jvalue)
// {
//     int reply;
//     std::vector<float> jnt;
//     std::vector<float> cart;
//     jvalue.resize(JNUMBER);
//     reply = client.GetRobotPose(jnt,cart);
//     std::cout <<"joint:" << jnt.at(0) <<" " << jnt.at(1) << jnt.at(2) <<" " << jnt.at(3)<< jnt.at(4) <<" " << jnt.at(5) << std::endl;
//     for(int i=0;i<JNUMBER;i++)
//     jvalue[i]=jnt[i];
//     if (reply==0)
//     {
// //        pose.translation() = Eigen::Vector3f(cart.at(0),cart.at(1),cart.at(2));
// //        Eigen::Quaternionf m = Eigen::AngleAxisf(cart.at(3)/180*M_PI, Eigen::Vector3f::UnitX())
// //                            * Eigen::AngleAxisf(cart.at(4)/180*M_PI, Eigen::Vector3f::UnitY())
// //                            * Eigen::AngleAxisf(cart.at(5)/180*M_PI, Eigen::Vector3f::UnitZ());
// //        pose.linear() = m.matrix();
// //        std::cout << "rpy:" << cart.at(3) << "," << cart.at(4)<< "," << cart.at(5) << std::endl;
//         return 0;
//     }
//     else
//         return reply;
// }
