#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include "jaka.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using jaka::HelloRequest;
using jaka::HelloReply;
using jaka::Robot;

class JakaServiceImpl final : public Robot::Service {
	Status SayHello(ServerContext* context, const HelloRequest* request,
					HelloReply* reply) override {
		std::string prefix("Hello ");
		reply->set_name(prefix + request->name());
		return Status::OK;
	}
};

void RunServer() {
	std::string server_address("0.0.0.0:50051");
	JakaServiceImpl service;

	ServerBuilder builder;
	builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
	builder.RegisterService(&service);
	std::unique_ptr<Server> server(builder.BuildAndStart());
	std::cout << "Server listening on " << server_address << std::endl;

	server->Wait();
}

int main(int argc, char** argv) {
	RunServer();
	return 0;
}
