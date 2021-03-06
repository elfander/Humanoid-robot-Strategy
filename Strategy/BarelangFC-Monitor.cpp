#include "BarelangFC-Monitor.h"
#include <stdio.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h>

using namespace BarelangFC;
using namespace std;
///////////////////////////////////////////////////////////////////
///////////////////////open port Monitoring////////////////////////
///////////////////////////////////////////////////////////////////
void BarelangMonitor::initSendMonitor(){
	sockMonitor = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&broadcastMonAddr, 0, sizeof(broadcastMonAddr));
        //bzero((char *) &broadcastMonAddr, sizeof(broadcastMonAddr));
	broadcastMonAddr.sin_family = AF_INET;
	broadcastMonAddr.sin_addr.s_addr = inet_addr(BROADCAST_IP);
	broadcastMonAddr.sin_port = htons(BROADCAST_PORT);
	sendStringLen = strlen(dataMonitor);
}

void BarelangMonitor::sendMonitor(int id, int cX, int cY, int vA, double vT, int nS, int bX, int bY){
	//usleep(500000);
	sprintf(dataMonitor, "%d,%d,%d,%d,%d,%d,%d,%d", id, int(cX), int(cY), vA, int(vT), nS, bX, bY);
	//sendto(sockMonitor, dataMonitor, strlen(dataMonitor), 0, (struct sockaddr *) &broadcastMonAddr, sizeof(broadcastMonAddr));
	//printf("   dataMonitor = %s\n",dataMonitor);
}

void BarelangMonitor::sendingMonitor(){
	//initSendMonitor();
	while(1){
		usleep(500000);
		sendto(sockMonitor, dataMonitor, strlen(dataMonitor), 0, (struct sockaddr *) &broadcastMonAddr, sizeof(broadcastMonAddr));
		//printf("   dataMonitor = %s\n",dataMonitor);
	}
}


