#ifndef WATCH_POSE_INFO_H
#define ROBOT_H

#include <eigen3/Eigen/Geometry>
#include "robot_client.h"
#define JNUMBER 6
class Robot
{
public:
    int start();
    int stop();
    int go(Eigen::Matrix4f targetPosition, double velocity);
    int asyncGo(Eigen::Matrix4f targetPosition, double velocity);
    int goRelative(Eigen::Matrix4f targetPosition, double velocity);
    int goHome();
    int jointMove(std::vector<float> pos_vector, float velocity);
    int ServoJEnable(bool ena);
    int servoJMove(std::vector<float> pos_vector,float t);
    // int getPose(std::vector<float> &jvalue);
    int controlIO(int n, bool status);
    Robot(std::string);
    ~Robot();

private:
    RobotClient client;
};

#endif // ROBOT_H
