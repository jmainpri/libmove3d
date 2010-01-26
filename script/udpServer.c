//Udp server to test BioMove3d behaviours. to compile : gcc UdpServer.c -o UdpServer

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

#define BUFLEN 512
#define NPACK 10
#define PORT 9930

int main(void)
{
  struct sockaddr_in si_me, si_other;
  int s, i, slen=sizeof(si_other);
  char buf[BUFLEN];

  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
	  perror("socket");
	  return 1;
}

  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr*) &si_me, sizeof(si_me))==-1){
	perror("bind");
	  return 1;
}
  
   do{
    memset(buf, 0, BUFLEN);
    if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*) &si_other, &slen)==-1){
	perror("receve");
	return 1;
    }
    printf("%s\n", buf);
  }while(strcmp(buf, "Exit"));
  sprintf(buf, "Stop Flooding me like this\n");
  if (sendto(s, buf, BUFLEN, 0, (struct sockaddr*)&si_other, slen)==-1){
    perror("send");
	return 1;
  }
  close(s);
  return 0;
}
