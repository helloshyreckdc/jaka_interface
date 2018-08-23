#ifndef __ROBOT_CLIENT_H_
#define __ROBOT_CLIENT_H_

#include <string>
#include <vector>
#include <memory>
#include <grpcpp/grpcpp.h>
#include "jaka.grpc.pb.h"

#define STATE_ESTOP         1
#define STATE_ESTOPREST     2
#define STATE_PWROFF        3
#define STATE_PWRON         4

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using jaka::HelloRequest;
using jaka::HelloReply;
using jaka::LogReq;
using jaka::ExeReply;
using jaka::EnumType;
using jaka::MovJReq;
using jaka::jointpos;
using jaka::cartpos;
using jaka::InvKineReq;
using jaka::InvKineReply;
using jaka::FwdKineReq;
using jaka::FwdKineReply;
using jaka::ServoJReq;
using jaka::BoolState;
//using jaka::RobotPoseReply;
using jaka::JAKAEmpty;

class RobotClient {
  public:
    RobotClient(std::string);

	std::string SayHello(const std::string& user);
	std::string LogIn(std::string username, std::string password);
	std::string LogOut(std::string username, std::string password);
	std::string StateChange(int state);
    int MovJ(std::vector<float> pos_vector, float speed);
    int ServojopEnable(bool ena);
    int ServoJ(std::vector<float> pos_vector, float t);

	//int GetRobotPose(std::vector<float>&jnt, std::vector<float>&cart);
    int KineInverse(std::vector<float> cart_vector, std::vector<float> curr_joint, std::vector<float>&joints);



  private:
    std::unique_ptr<jaka::Robot::Stub> stub_;
};

#endif
