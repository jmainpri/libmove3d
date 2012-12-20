#include "UdpClient.h"
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

UdpClient* globalUdpClient = NULL;
//public
//Constructors and destructors
UdpClient::UdpClient(std::string serverIp, int port){
  _serverIp = serverIp;
  _port = port;
  if ((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    perror("socket");

  _si_other.sin_family = AF_INET;
  _si_other.sin_port = htons(_port);
  if (inet_aton(_serverIp.c_str(), &_si_other.sin_addr)==0) {
    std::cerr << "inet_aton() failed" << std::endl;
  }
}

UdpClient::~UdpClient(){
  close(_socket);
}

//functions
//TODO
// void UdpClient::setBlocant(bool block){
//   if(block){//Blocant process
//     int status = fcntl(_socket, F_GETFL);
//     fcntl(_socket, F_SETFL, status &~ O_NONBLOCK);
//   }else{//Non blocant process
//     fcntl(_socket, F_SETFL, O_NONBLOCK);
//   }
// }

void UdpClient::send(std::string& message){
  int slen = sizeof(_si_other);
  if (sendto(_socket, message.c_str(), message.size(), 0, (struct sockaddr*)&_si_other, slen)==-1)
    perror("sendto()");
}
std::string UdpClient::receive(void){
  struct sockaddr_in si_other;
  int slen = sizeof(si_other);
  char buf[512];
  std::string message;

  switch (recvfrom(_socket, buf, 512, 0, (struct sockaddr*)&si_other, (socklen_t*)&slen)){
    case -1:{
      perror("recvfrom()");
      message.assign("");
    }
    case EAGAIN:{
      message.assign("");
    }
    default:{
      message.assign(buf);
      break;
    }
  }
  return message;
}

////////// Move3d Functions //////////

void UdpClient::sendConfig(double * config, int size){
  for(int i = 0; i < size; i++){
    std::string jntValue(convertToString(config[i]));
    this->send(jntValue);
  }
}

////////// Move3d Functions //////////
