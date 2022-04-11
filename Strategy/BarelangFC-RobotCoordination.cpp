#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "BarelangFC-RobotCoordination.h"


#define SERVER1 "192.168.8.101"  //IP robot 1
#define SERVER2 "192.168.8.102"  //IP robot 2
#define SERVER3 "192.168.8.103"  //IP robot 3
#define SERVER4 "192.168.8.104"  //IP robot 4
#define SERVER5 "192.168.8.105"  //IP robot 5

//#define SERVER1 "172.17.0.13"  //IP robot 1
//#define SERVER2 "172.17.0.10"  //IP robot 2
//#define SERVER3 "172.17.0.15"  //IP robot 3
//#define SERVER4 "172.17.0.14"  //IP robot 4
//#define SERVER5 "172.17.0.11"  //IP robot 5

//#define SERVER1 "172.17.10.1"  //IP robot 1
//#define SERVER2 "172.17.10.2"  //IP robot 2
//#define SERVER3 "172.17.10.3"  //IP robot 3
//#define SERVER4 "172.17.10.4"  //IP robot 4
//#define SERVER5 "172.17.10.5"  //IP robot 5


using namespace BarelangFC;

static int 	socketRobotCoordinationIn1 = 0,
		socketRobotCoordinationIn2 = 0,
		socketRobotCoordinationIn3 = 0,
		socketRobotCoordinationIn4 = 0,
		socketRobotCoordinationIn5 = 0,
		socketRobotCoordinationOut = 0;

struct sockaddr_in local_addr1, dest_addr, dest_addr1, dest_addr2, dest_addr3, dest_addr4, dest_addr5;
struct sockaddr_in local_addr2;
struct sockaddr_in local_addr3;
struct sockaddr_in local_addr4;
struct sockaddr_in local_addr5;

//bool	useBroadcast = true;
bool	useBroadcast = false;

int RobotCoordination::dataRecieve(uint8 * data, int size) {
	int result = 0;
	int compare = (0x80 << ((size-1)*8));
	for (int i = 0 ; i< size ; i++) {
		result = (result<<8)|*(data+i);
	}
	result = (result & compare) == compare? (result & (~compare))*-1: result& (~compare);
	return result;
}

void RobotCoordination::exitCommunication() {
	if (RobotID != 1) { if (socketRobotCoordinationIn1 > 0) close(socketRobotCoordinationIn1); }
	if (RobotID != 2) { if (socketRobotCoordinationIn2 > 0) close(socketRobotCoordinationIn2); }
	if (RobotID != 3) { if (socketRobotCoordinationIn3 > 0) close(socketRobotCoordinationIn3); }
	if (RobotID != 4) { if (socketRobotCoordinationIn4 > 0) close(socketRobotCoordinationIn4); }
	if (RobotID != 5) { if (socketRobotCoordinationIn5 > 0) close(socketRobotCoordinationIn5); }
	if (socketRobotCoordinationOut > 0) close(socketRobotCoordinationOut);
}

void RobotCoordination::initCommunicationIn1() {
	socketRobotCoordinationIn1 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationIn1 < 0) printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");

	bzero((char *) &local_addr1, sizeof(local_addr1));
	local_addr1.sin_family = AF_INET;
	if (useBroadcast) {
		local_addr1.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	} else {
		local_addr1.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	local_addr1.sin_port = htons(ROBOT_COORDINATION_PORT_IN_1);

	if (bind(socketRobotCoordinationIn1, (struct sockaddr *) &local_addr1, sizeof(local_addr1)) < 0) printf("BarelangFC-RobotCoordination => Could not bind to port 1\n");

	int broadcast = 1;
	if ((setsockopt(socketRobotCoordinationIn1,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) perror("setsockopt");

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationIn1, F_GETFL, 0);
	if (flags == -1) flags = 0;
	if (fcntl(socketRobotCoordinationIn1, F_SETFL, flags | O_NONBLOCK) < 0) { }
}

void RobotCoordination::initCommunicationIn2() {
	socketRobotCoordinationIn2 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationIn2 < 0) printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");

	bzero((char *) &local_addr2, sizeof(local_addr2));
	local_addr2.sin_family = AF_INET;
	if (useBroadcast) {
		local_addr2.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	} else {
		local_addr2.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	local_addr2.sin_port = htons(ROBOT_COORDINATION_PORT_IN_2);

	if (bind(socketRobotCoordinationIn2, (struct sockaddr *) &local_addr2, sizeof(local_addr2)) < 0) printf("BarelangFC-RobotCoordination => Could not bind to port 2\n");

	int broadcast = 1;
	if ((setsockopt(socketRobotCoordinationIn2,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) perror("setsockopt");

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationIn2, F_GETFL, 0);
	if (flags == -1) flags = 0;
	if (fcntl(socketRobotCoordinationIn2, F_SETFL, flags | O_NONBLOCK) < 0) { }
}

void RobotCoordination::initCommunicationIn3() {
	socketRobotCoordinationIn3 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationIn3 < 0) printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");

	bzero((char *) &local_addr3, sizeof(local_addr3));
	local_addr3.sin_family = AF_INET;
	if (useBroadcast) {
		local_addr3.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	} else {
		local_addr3.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	local_addr3.sin_port = htons(ROBOT_COORDINATION_PORT_IN_3);

	if (bind(socketRobotCoordinationIn3, (struct sockaddr *) &local_addr3, sizeof(local_addr3)) < 0) printf("BarelangFC-RobotCoordination => Could not bind to port 3\n");

	int broadcast = 1;
	if ((setsockopt(socketRobotCoordinationIn3,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) perror("setsockopt");

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationIn3, F_GETFL, 0);
	if (flags == -1) flags = 0;
	if (fcntl(socketRobotCoordinationIn3, F_SETFL, flags | O_NONBLOCK) < 0) { }
}

void RobotCoordination::initCommunicationIn4() {
	socketRobotCoordinationIn4 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationIn4 < 0) printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");

	bzero((char *) &local_addr4, sizeof(local_addr4));
	local_addr4.sin_family = AF_INET;
	if (useBroadcast) {
		local_addr4.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	} else {
		local_addr4.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	local_addr4.sin_port = htons(ROBOT_COORDINATION_PORT_IN_4);

	if (bind(socketRobotCoordinationIn4, (struct sockaddr *) &local_addr4, sizeof(local_addr4)) < 0) printf("BarelangFC-RobotCoordination => Could not bind to port 4\n");

	int broadcast = 1;
	if ((setsockopt(socketRobotCoordinationIn4,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) perror("setsockopt");

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationIn4, F_GETFL, 0);
	if (flags == -1) flags = 0;
	if (fcntl(socketRobotCoordinationIn4, F_SETFL, flags | O_NONBLOCK) < 0) { }
}

void RobotCoordination::initCommunicationIn5() {
	socketRobotCoordinationIn5 = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationIn5 < 0) printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");

	bzero((char *) &local_addr5, sizeof(local_addr5));
	local_addr5.sin_family = AF_INET;
	if (useBroadcast) {
		local_addr5.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	} else {
		local_addr5.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	local_addr5.sin_port = htons(ROBOT_COORDINATION_PORT_IN_5);

	if (bind(socketRobotCoordinationIn5, (struct sockaddr *) &local_addr5, sizeof(local_addr5)) < 0) printf("BarelangFC-RobotCoordination => Could not bind to port 5\n");

	int broadcast = 1;
	if ((setsockopt(socketRobotCoordinationIn5,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) perror("setsockopt");

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationIn5, F_GETFL, 0);
	if (flags == -1) flags = 0;
	if (fcntl(socketRobotCoordinationIn5, F_SETFL, flags | O_NONBLOCK) < 0) { }
}

void RobotCoordination::initCommunicationIn() {
	if (RobotID != 1) { initCommunicationIn1(); }
	if (RobotID != 2) { initCommunicationIn2(); }
	if (RobotID != 3) { initCommunicationIn3(); }
	if (RobotID != 4) { initCommunicationIn4(); }
	if (RobotID != 5) { initCommunicationIn5(); }
}

void RobotCoordination::readRobotCoordinationData1() {
	const int MAX_LENGTH = 50;
	static char data[MAX_LENGTH];
	socklen_t local_addr1_len = sizeof(local_addr1);
	int len = recvfrom(socketRobotCoordinationIn1, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr1, &local_addr1_len);
	while (len > 0) {
		if (memcmp(data, ROBOT_COORDINATION_STRUCT_HEADER, sizeof(ROBOT_COORDINATION_STRUCT_HEADER) - 1) == 0) {
			memcpy(&coordinationData, data, sizeof(BarelangFCCoordinationData));
		}
		len = recvfrom(socketRobotCoordinationIn1, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr1, &local_addr1_len);
	}

	if (coordinationData.robotNumber == 1) {
		robot1Id	= coordinationData.robotNumber;
		robot1Status	= coordinationData.robotStatus;
		robot1State	= dataRecieve(&coordinationData.stateNumber[0], sizeof(coordinationData.stateNumber));
		robot1GridPosition	= coordinationData.robotGrid;
		robot1FBall	= coordinationData.foundBall;
		robot1DBall	= dataRecieve(&coordinationData.distanceBall[0], sizeof(coordinationData.stateNumber));
		robot1GridBall	= coordinationData.ballGrid;
		robot1BackIn    = coordinationData.backIn;
	} else {
		robot1DBall = 232;
	}
}

void RobotCoordination::readRobotCoordinationData2() {
	const int MAX_LENGTH = 50;
	static char data[MAX_LENGTH];
	socklen_t local_addr2_len = sizeof(local_addr2);
	int len = recvfrom(socketRobotCoordinationIn2, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr2, &local_addr2_len);
	while (len > 0) {
		if (memcmp(data, ROBOT_COORDINATION_STRUCT_HEADER, sizeof(ROBOT_COORDINATION_STRUCT_HEADER) - 1) == 0) {
			memcpy(&coordinationData, data, sizeof(BarelangFCCoordinationData));
		}
		len = recvfrom(socketRobotCoordinationIn2, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr2, &local_addr2_len);
	}

	if (coordinationData.robotNumber == 2) {
		robot2Id	= coordinationData.robotNumber;
		robot2Status	= coordinationData.robotStatus;
		robot2State	= dataRecieve(&coordinationData.stateNumber[0], sizeof(coordinationData.stateNumber));
		robot2GridPosition	= coordinationData.robotGrid;
		robot2FBall	= coordinationData.foundBall;
		robot2DBall	= dataRecieve(&coordinationData.distanceBall[0], sizeof(coordinationData.stateNumber));
		robot2GridBall	= coordinationData.ballGrid;
		robot2BackIn    = coordinationData.backIn;
	} else {
		robot2DBall = 232;
	}
}
void RobotCoordination::readRobotCoordinationData3() {
	const int MAX_LENGTH = 50;
	static char data[MAX_LENGTH];
	socklen_t local_addr3_len = sizeof(local_addr3);
	int len = recvfrom(socketRobotCoordinationIn3, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr3, &local_addr3_len);
	while (len > 0) {
		if (memcmp(data, ROBOT_COORDINATION_STRUCT_HEADER, sizeof(ROBOT_COORDINATION_STRUCT_HEADER) - 1) == 0) {
			memcpy(&coordinationData, data, sizeof(BarelangFCCoordinationData));
		}
		len = recvfrom(socketRobotCoordinationIn3, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr3, &local_addr3_len);
	}

	if (coordinationData.robotNumber == 3) {
		robot3Id	= coordinationData.robotNumber;
		robot3Status	= coordinationData.robotStatus;
		robot3State	= dataRecieve(&coordinationData.stateNumber[0], sizeof(coordinationData.stateNumber));
		robot3GridPosition	= coordinationData.robotGrid;
		robot3FBall	= coordinationData.foundBall;
		robot3DBall	= dataRecieve(&coordinationData.distanceBall[0], sizeof(coordinationData.stateNumber));
		robot3GridBall	= coordinationData.ballGrid;
                robot3BackIn    = coordinationData.backIn;
	} else {
		robot3DBall = 232;
	}
}

void RobotCoordination::readRobotCoordinationData4() {
	const int MAX_LENGTH = 50;
	static char data[MAX_LENGTH];
	socklen_t local_addr4_len = sizeof(local_addr4);
	int len = recvfrom(socketRobotCoordinationIn4, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr4, &local_addr4_len);
	while (len > 0) {
		if (memcmp(data, ROBOT_COORDINATION_STRUCT_HEADER, sizeof(ROBOT_COORDINATION_STRUCT_HEADER) - 1) == 0) {
			memcpy(&coordinationData, data, sizeof(BarelangFCCoordinationData));
		}
		len = recvfrom(socketRobotCoordinationIn4, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr4, &local_addr4_len);
	}

	if (coordinationData.robotNumber == 4) {
		robot4Id	= coordinationData.robotNumber;
		robot4Status	= coordinationData.robotStatus;
		robot4State	= dataRecieve(&coordinationData.stateNumber[0], sizeof(coordinationData.stateNumber));
		robot4GridPosition	= coordinationData.robotGrid;
		robot4FBall	= coordinationData.foundBall;
		robot4DBall	= dataRecieve(&coordinationData.distanceBall[0], sizeof(coordinationData.stateNumber));
		robot4GridBall	= coordinationData.ballGrid;
		robot4BackIn    = coordinationData.backIn;
	} else {
		robot4DBall = 232;
	}
}

void RobotCoordination::readRobotCoordinationData5() {
	const int MAX_LENGTH = 50;
	static char data[MAX_LENGTH];
	socklen_t local_addr5_len = sizeof(local_addr5);
	int len = recvfrom(socketRobotCoordinationIn5, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr5, &local_addr5_len);
	while (len > 0) {
		if (memcmp(data, ROBOT_COORDINATION_STRUCT_HEADER, sizeof(ROBOT_COORDINATION_STRUCT_HEADER) - 1) == 0) {
			memcpy(&coordinationData, data, sizeof(BarelangFCCoordinationData));
		}
		len = recvfrom(socketRobotCoordinationIn5, data, MAX_LENGTH, 0, (struct sockaddr *) &local_addr5, &local_addr5_len);
	}

	if (coordinationData.robotNumber == 5) {
		robot5Id	= coordinationData.robotNumber;
		robot5Status	= coordinationData.robotStatus;
		robot5State	= dataRecieve(&coordinationData.stateNumber[0], sizeof(coordinationData.stateNumber));
		robot5GridPosition	= coordinationData.robotGrid;
		robot5FBall	= coordinationData.foundBall;
		robot5DBall	= dataRecieve(&coordinationData.distanceBall[0], sizeof(coordinationData.stateNumber));
		robot5GridBall	= coordinationData.ballGrid;
		robot5BackIn    = coordinationData.backIn;
	} else {
		robot5DBall = 232;
	}
}

void RobotCoordination::readRobotCoordinationData() {
	if (refresh) { //400
		//printf("\n..................................................................................|||\n");
		coordinationData.robotNumber 	= 0;
		coordinationData.robotStatus	= 0;
		coordinationData.stateNumber[0]	= 0;
		coordinationData.stateNumber[1]	= 0;
		coordinationData.robotGrid	= 0;
		coordinationData.foundBall	= 0;
		coordinationData.distanceBall[0] = 232;
		coordinationData.distanceBall[1] = 0;
		coordinationData.ballGrid	= 0;
		//coordinationData.backIn         = 0;
	} else {
		if (RobotID != 1) { readRobotCoordinationData1(); }
		if (RobotID != 2) { readRobotCoordinationData2(); }
		if (RobotID != 3) { readRobotCoordinationData3(); }
		if (RobotID != 4) { readRobotCoordinationData4(); }
		if (RobotID != 5) { readRobotCoordinationData5(); }
	}

	//printf("  Read = \n");
	//printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot1Id, robot1FBall, robot1DBall, robot1State);
	//printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot2Id, robot2FBall, robot2DBall, robot2State);
	//printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot3Id, robot3FBall, robot3DBall, robot3State);
	//printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot4Id, robot4FBall, robot4DBall, robot4State);
	//printf("  BarelangFC%d : Found = %d \t Dis = %d \t State = %d\n", robot5Id, robot5FBall, robot5DBall, robot5State);

	//New
	//printf("  BarelangFC%d : Role = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDis = %d\tGridBall = %d\n", robot1Id, robot1Role, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall);
	//printf("  BarelangFC%d : Role = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDis = %d\tGridBall = %d\n", robot2Id, robot2Role, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall);
	//printf("  BarelangFC%d : Role = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDis = %d\tGridBall = %d\n", robot3Id, robot3Role, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall);
	//printf("  BarelangFC%d : Role = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDis = %d\tGridBall = %d\n", robot4Id, robot4Role, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall);
	//printf("  BarelangFC%d : Role = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDis = %d\tGridBall = %d\n", robot5Id, robot5Role, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////SEND DATA OUT /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RobotCoordination::initCommunicationOut() {
	socketRobotCoordinationOut = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketRobotCoordinationOut < 0) {
		printf("BarelangFC-RobotCoordination => Could not open datagram socket\n");
	}

	if (RobotID == 1) { ROBOT_COORDINATION_PORT_OUT = 5041; }
	else if (RobotID == 2) { ROBOT_COORDINATION_PORT_OUT = 5042; }
	else if (RobotID == 3) { ROBOT_COORDINATION_PORT_OUT = 5043; }
	else if (RobotID == 4) { ROBOT_COORDINATION_PORT_OUT = 5044; }
	else if (RobotID == 5) { ROBOT_COORDINATION_PORT_OUT = 5045; }

	if (useBroadcast) {
		bzero((char *) &dest_addr, sizeof(dest_addr));
		dest_addr.sin_family = AF_INET;
		dest_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
		dest_addr.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

		if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr, sizeof(dest_addr)) < 0) {
			printf("BarelangFC-RobotCoordination => Could not bind to port\n");
		}
	} else {
		if (RobotID != 1) {
			bzero((char *) &dest_addr1, sizeof(dest_addr1));
			dest_addr1.sin_family = AF_INET;
			dest_addr1.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

			if (inet_aton(SERVER1 , &dest_addr1.sin_addr) == 0) {
				fprintf(stderr, "inet_aton() failed\n");
				exit(1);
			}

			//if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr1, sizeof(dest_addr1)) < 0) {
			//	printf("BarelangFC-RobotCoordination => Could not bind to port 1\n");
			//}
		}

		if (RobotID != 2) {
			bzero((char *) &dest_addr2, sizeof(dest_addr2));
			dest_addr2.sin_family = AF_INET;
			dest_addr2.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

			if (inet_aton(SERVER2 , &dest_addr2.sin_addr) == 0) {
				fprintf(stderr, "inet_aton() failed\n");
				exit(1);
			}

			//if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr2, sizeof(dest_addr2)) < 0) {
			//	printf("BarelangFC-RobotCoordination => Could not bind to port 2\n");
			//}
		}

		if (RobotID != 3) {
			bzero((char *) &dest_addr3, sizeof(dest_addr3));
			dest_addr3.sin_family = AF_INET;
			dest_addr3.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

			if (inet_aton(SERVER3 , &dest_addr3.sin_addr) == 0) {
				fprintf(stderr, "inet_aton() failed\n");
				exit(1);
			}

			//if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr3, sizeof(dest_addr3)) < 0) {
			//	printf("BarelangFC-RobotCoordination => Could not bind to port 3\n");
			//}
		}

		if (RobotID != 4) {
			bzero((char *) &dest_addr4, sizeof(dest_addr4));
			dest_addr4.sin_family = AF_INET;
			dest_addr4.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

			if (inet_aton(SERVER4 , &dest_addr4.sin_addr) == 0) {
				fprintf(stderr, "inet_aton() failed\n");
				exit(1);
			}

			//if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr4, sizeof(dest_addr4)) < 0) {
			//	printf("BarelangFC-RobotCoordination => Could not bind to port 4\n");
			//}
		}


		if (RobotID != 5) {
			bzero((char *) &dest_addr5, sizeof(dest_addr5));
			dest_addr5.sin_family = AF_INET;
			dest_addr5.sin_port = htons(ROBOT_COORDINATION_PORT_OUT);

			if (inet_aton(SERVER5 , &dest_addr5.sin_addr) == 0) {
				fprintf(stderr, "inet_aton() failed\n");
				exit(1);
			}

			//if (bind(socketRobotCoordinationOut, (struct sockaddr *) &dest_addr5, sizeof(dest_addr5)) < 0) {
			//	printf("BarelangFC-RobotCoordination => Could not bind to port 5\n");
			//}
		}
	}

	int broadcast = 1;
	if((setsockopt(socketRobotCoordinationOut,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)))) {
		perror("setsockopt");
	}

	// Nonblocking receive:
	int flags = fcntl(socketRobotCoordinationOut, F_GETFL, 0);
	if (flags == -1) {
		flags = 0;
	}
}

void RobotCoordination::sendRobotCoordinationData(int rNumber, int rStatus, int sNumber, int gridPos, int fBall, int dBall, int gridBall, int backIn) {
	//0x8000 digunakan sebagai penanda (signed) pada tipe data int
	sNumber = sNumber < 0? -sNumber|0x8000:sNumber;
	char dataToSend[20] ="BarelangFC";
	dataToSend[10]	= rNumber;
	dataToSend[11]	= rStatus;
	dataToSend[12]	= sNumber>>8;
	dataToSend[13]	= sNumber&0xff;
	dataToSend[14]	= gridPos;
	dataToSend[15]	= fBall;
	dataToSend[16]	= dBall>>8;
	dataToSend[17]	= dBall&0xff;
	dataToSend[18]	= gridBall;
	dataToSend[19]  = backIn;
	if (useBroadcast) {
		sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr, sizeof(dest_addr));
	} else {
		if (RobotID != 1) { sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr1, sizeof(dest_addr1)); }
		if (RobotID != 2) { sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr2, sizeof(dest_addr2)); }
		if (RobotID != 3) { sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr3, sizeof(dest_addr3)); }
		if (RobotID != 4) { sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr4, sizeof(dest_addr4)); }
		if (RobotID != 5) { sendto(socketRobotCoordinationOut ,dataToSend, 20 , 0 , (struct sockaddr *) &dest_addr5, sizeof(dest_addr5)); }
	}

	//printf("  Sent = \n");
	//printf("  BarelangFC%d __ Found = %d \t Dis = %d \t State = %d\n\n", rNumber, fBall, dBall, sNumber);
	//printf("   BarelangFC%d \tRole = %d\tStatus = %d\tState = %d\tGridPos = %d\tFound = %d\tDist.Ball = %d\tGridBall = %d\n",rNumber, rRole, rStatus, sNumber, gridPos, fBall, dBall, gridBall);
}
