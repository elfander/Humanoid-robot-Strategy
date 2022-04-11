#ifndef BARELANGFC_MONITOR_H_
#define BARELANGFC_MONITOR_H_

#include <stdio.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h>

//MONITORING
#define BROADCAST_IP "192.168.8.173"
//#define BROADCAST_IP "192.168.8.163"
#define BROADCAST_PORT 8010
#define MAX_LEN 1024

namespace BarelangFC {
	class BarelangMonitor{
		private :
		//Socket Monitor C#
		int sockMonitor;
		struct sockaddr_in broadcastMonAddr;
		unsigned int sendStringLen;
		char dataMonitor[100];

		public :
		int numBot,valAng,numState;
		double crdX,crdY,valTar;
		void initSendMonitor();
		void sendingMonitor();
		void sendMonitor(int id, int cX, int cY, int vA, double vT, int nS, int bX, int bY);
	};
}
#endif

