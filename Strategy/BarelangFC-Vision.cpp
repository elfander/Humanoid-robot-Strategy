#include "BarelangFC-Vision.h"

using namespace BarelangFC;
using namespace std;
using namespace cv;


#define PORT_DARKNET 2000  	//The port on which to listen for incoming data
struct sockaddr_in si_me, si_other;
socklen_t slen;
int s, recv_len;
char myWindow[] = "TrackBall - PID Tuner";

/*#define BUFLEN 30  		//Max length of buffer
char recvCoor[BUFLEN];
FILE *camNotePad;
*/

void BarelangVision::help() {
	printf("\n\t\t----------BarelangFC-Vision-----------");
	printf("\n\t\tHelp ----------------------------- [H]");
	printf("\n\t\tParse Ball ----------------------- [B]");
	printf("\n\t\tParse Field ---------------------- [F]");
	printf("\n\t\tParse Goal ----------------------- [G]");
	printf("\n\t\tSave Config ---------------------- [S]");
	printf("\n\t\tLoad Config ---------------------- [L]");
	printf("\n\t\tExit BarelangFC-Vision ----------- [ESC]");
	printf("\n\n");
}

void BarelangVision::die(char *s) {
    perror(s);
    exit(1);
}

void BarelangVision::initialize() {
	//slen = sizeof(si_other);
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		die("socket");
	}
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT_DARKNET);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	//bind socket to port
	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1) {
		die("bind");
	}

}

void BarelangVision::processing() {
	initialize();
        //printf("Waiting for data...");
//        fflush(stdout);
        //try to receive some data, this is a blocking call
/*        if ((recv_len = recvfrom(s, recvCoor, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
        {
            die("recvfrom()");
        }
*/        //print details of the client/peer and the data received
        //printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));

	while(1) {
//		sleep(1);
		//printf("Waiting for data...");
		fflush(stdout);
		bzero(recvCoor,sizeof(recvCoor));
		//try to receive some data, this is a blocking call
		if ((recv_len = recvfrom(s, recvCoor, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1) {
			die("recvfrom()");
		}
//		fflush(stdout);

		//print details of the client/peer and the data received
		//printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
/*		usleep(40000); //40000
		camNotePad = fopen("/home/ubuntu/robot/darknet/CameraNote", "r");
		fscanf(camNotePad, "%s", recvCoor);
		fclose(camNotePad);
*/
		//printf("received = %s\n" , recvCoor);
		indexing = 0;
		parseCoor = strtok (recvCoor,",");
		while (parseCoor!=NULL) {
			indexing++;
			if (indexing == 1)	{ sscanf(parseCoor,"%d", &Ball_X); }
			else if (indexing == 2)	{ sscanf(parseCoor,"%d", &Ball_Y); }
			else if (indexing == 3)	{ sscanf(parseCoor,"%d", &Ball_W); }
			else if (indexing == 4)	{ sscanf(parseCoor,"%d", &Ball_H); }
			else if (indexing == 5)	{ sscanf(parseCoor,"%d", &Ball_D); }
			else if (indexing == 6)	{ sscanf(parseCoor,"%d", &Goal_X); }
			else if (indexing == 7)	{ sscanf(parseCoor,"%d", &Goal_Y); }
			else if (indexing == 8)	{ sscanf(parseCoor,"%d", &Goal_LH); }
			else if (indexing == 9)	{ sscanf(parseCoor,"%d", &Goal_RH); }
			else if (indexing == 10) { sscanf(parseCoor,"%d", &Goal_C); }
			else if (indexing == 11) { sscanf(parseCoor,"%d", &Goal_LD); }
			else if (indexing == 12) { sscanf(parseCoor,"%d", &Goal_RD); }
			parseCoor = strtok (NULL,",");
		}
		//if (Goal_X != -1 || Goal_Y !=-1) Goal = 1;
		//else Goal = 0;
	        //printf("\nBola\t= %d \t %d \nGawang\t= %d \t %d\n" , Ball_X, Ball_Y, Goal_X, Goal_Y);
	}
	//printf("received = %d,%d\n" , Ball_X, Ball_Y);

//	printf("\nball-x = %d \t ball-y = %d \n" , Ball_X, Ball_Y);
}

void BarelangVision::pidTuner() {

	iSliderPanKP = iSliderTiltKP = 1;
	namedWindow(myWindow, WINDOW_NORMAL);
	createTrackbar("Pan KP", myWindow, &iSliderPanKP, 100);
	createTrackbar("Pan KI", myWindow, &iSliderPanKI, 10);
	createTrackbar("Pan KD", myWindow, &iSliderPanKD, 1000);
	createTrackbar("Tilt KP", myWindow, &iSliderTiltKP, 100);
	createTrackbar("Tilt KI", myWindow, &iSliderTiltKI, 10);
	createTrackbar("Tilt KD", myWindow, &iSliderTiltKD, 1000);
//	printf("init\n");

	while(1) {
		panKP = (double)iSliderPanKP / 100;
		panKI = (double)iSliderPanKI / 10;
		panKD = (double)iSliderPanKD / 10000000;

		tiltKP = (double)iSliderTiltKP / 100;
		tiltKI = (double)iSliderTiltKI / 10;
		tiltKD = (double)iSliderTiltKD / 10000000;
		printf("KP Pan : %f\t", panKP);
		printf("KI Pan : %f\t", panKI);
		printf("KD Pan : %f\n", panKD);
		printf("KP Tilt : %f\t", tiltKP);
		printf("KI Tilt : %f\t", tiltKI);
		printf("KD Tilt : %f\n\n\n", tiltKD);
                int iKey = waitKey(50);
	}
}
