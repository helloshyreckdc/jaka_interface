#include <iostream>

#include "unistd.h"
#include "robot_client.h"

RobotClient::RobotClient(std::string address)
    : stub_(jaka::Robot::NewStub(grpc::CreateChannel(
                address, grpc::InsecureChannelCredentials()))) {}

std::string RobotClient::SayHello(const std::string& user) {
        HelloRequest request;
        request.set_name(user);

        HelloReply reply;
        ClientContext context;
        Status status = stub_->SayHello(&context, request, &reply);

        if (status.ok()) {
                return reply.name();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                  << std::endl;
                return "RPC failed";
        }
}

std::string RobotClient::LogIn(std::string username, std::string password) {
        LogReq req;
        req.set_username(username);
        req.set_password(password);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->LogIn(&context, req, &reply);

        if (status.ok()) {
                return reply.errorstring();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                  << std::endl;
                return "RPC failed";
        }
}

std::string RobotClient::LogOut(std::string username, std::string password) {
        LogReq req;
        req.set_username(username);
        req.set_password(password);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->LogOut(&context, req, &reply);

        if (status.ok()) {
                return reply.errorstring();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                  << std::endl;
                return "RPC failed";
        }
}

std::string RobotClient::StateChange(int state) {
        EnumType enumreq;
        enumreq.set_typeid_(state);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->StateChange(&context, enumreq, &reply);

        if (status.ok()) {
                return reply.errorstring();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                  << std::endl;
                return "RPC failed";
        }
}

int RobotClient::MovJ(std::vector<float> pos_vector, float speed) {
        MovJReq req;
        jointpos *pos = new jointpos;

        assert(pos_vector.size()==6);

        pos->set_joint1(pos_vector.at(0));
        pos->set_joint2(pos_vector.at(1));
        pos->set_joint3(pos_vector.at(2));
        pos->set_joint4(pos_vector.at(3));
        pos->set_joint5(pos_vector.at(4));
        pos->set_joint6(pos_vector.at(5));
        req.set_allocated_dest(pos);
        req.set_speed(speed);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->MovJ(&context, req, &reply);

        if (status.ok()) {
                std::cout << reply.errorstring();
                return reply.execode();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                  << std::endl;
                return -1;
        }
}

int RobotClient::ServojopEnable(bool ena)
{
        BoolState req;
        req.set_boolval(1);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->ServoJOpEnable(&context, req, &reply);

        if (status.ok()) {
                std::cout << reply.errorstring();
                return reply.execode();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                        << std::endl;
                return -1;
        }

}
int RobotClient::ServoJ(std::vector<float> pos_vector, float t = 8)
{
        ServoJReq req;
        jointpos *pos = new jointpos;

        assert(pos_vector.size()==6);

        pos->set_joint1(pos_vector.at(0));
        pos->set_joint2(pos_vector.at(1));
        pos->set_joint3(pos_vector.at(2));
        pos->set_joint4(pos_vector.at(3));
        pos->set_joint5(pos_vector.at(4));
        pos->set_joint6(pos_vector.at(5));
        req.set_allocated_dest(pos);
        req.set_t(t);

        ExeReply reply;
        ClientContext context;
        Status status = stub_->ServoJ(&context, req, &reply);

        if (status.ok()) {
                std::cout << reply.errorstring();
                return reply.execode();
        } else {
                std::cout << status.error_code() << ": " << status.error_message()
                                        << std::endl;
                return -1;
        }
}


// int RobotClient::GetRobotPose(std::vector<float>&jnt,
//                                  std::vector<float>&cart) {
//         JAKAEmpty req;
//        // RobotPoseReply reply;

//         ClientContext context;
//         Status status = stub_->GetRobotPose(&context, req, &reply);

//         if (status.ok()) {
//                 jnt.clear();
//                 jnt.push_back(reply.jnt_pos().joint1());
//                 jnt.push_back(reply.jnt_pos().joint2());
//                 jnt.push_back(reply.jnt_pos().joint3());
//                 jnt.push_back(reply.jnt_pos().joint4());
//                 jnt.push_back(reply.jnt_pos().joint5());
//                 jnt.push_back(reply.jnt_pos().joint6());
//                 cart.clear();
//                 cart.push_back(reply.cart_pos().x());
//                 cart.push_back(reply.cart_pos().y());
//                 cart.push_back(reply.cart_pos().z());
//                 cart.push_back(reply.cart_pos().a());
//                 cart.push_back(reply.cart_pos().b());
//                 cart.push_back(reply.cart_pos().c());
//                 return 0;
//         } else {
//                 return -1;
//         }
// }


int RobotClient::KineInverse(std::vector<float> cart_vector,
                                     std::vector<float> curr_joint,
                                     std::vector<float> &joints) {
        InvKineReq req;
        cartpos *world = new cartpos;
        jointpos *currjoint = new jointpos;

        assert(cart_vector.size()==6);
        assert(curr_joint.size()==6);

        world->set_x(cart_vector.at(0));
        world->set_y(cart_vector.at(1));
        world->set_z(cart_vector.at(2));
        world->set_a(cart_vector.at(3));
        world->set_b(cart_vector.at(4));
        world->set_c(cart_vector.at(5));
        req.set_allocated_world(world);

        currjoint->set_joint1(curr_joint.at(0));
        currjoint->set_joint2(curr_joint.at(1));
        currjoint->set_joint3(curr_joint.at(2));
        currjoint->set_joint4(curr_joint.at(3));
        currjoint->set_joint5(curr_joint.at(4));
        currjoint->set_joint6(curr_joint.at(5));
        req.set_allocated_currjoint(currjoint);

        InvKineReply reply;
        ClientContext context;
        Status status = stub_->KineInverse(&context, req, &reply);

        if (status.ok()) {
            joints.clear();
            joints.push_back(reply.joints().joint1());
            joints.push_back(reply.joints().joint2());
            joints.push_back(reply.joints().joint3());
            joints.push_back(reply.joints().joint4());
            joints.push_back(reply.joints().joint5());
            joints.push_back(reply.joints().joint6());
            return reply.res();
        } else {
            std::cout << status.error_code() << ": " << status.error_message()
                      << std::endl;
            return -1;
        }

}

