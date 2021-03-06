#include "BarelangFC-Strategy.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>

using namespace BarelangFC;
using namespace std;


// Function To Set Timer =====================================================================
struct timeval  t1Imu,
                t2Imu;
double  elapsedTimeImu,
        secondImu;
bool    timerImu = false;
void BarelangStrategy::setWaktuImu() {
        elapsedTimeImu     =
        secondImu          = 0;
        timerImu           = false;

        gettimeofday(&t1Imu, NULL);
}

// Function For Check Timer
void BarelangStrategy::cekWaktuImu(double detik) {
        gettimeofday(&t2Imu, NULL);

        // compute and print the elapsed time in millisec
        elapsedTimeImu = (t2Imu.tv_sec - t1Imu.tv_sec) * 1000.0;
        elapsedTimeImu += (t2Imu.tv_usec - t1Imu.tv_usec) / 1000.0;
        secondImu = elapsedTimeImu / 1000.0;
        //printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

        if (secondImu >= detik) {
                timerImu = true;
        } else {
                timerImu = false;
        }
}

int BarelangStrategy::openPort(int comNumber){
	port = 0;
	if(comNumber==0) {
		port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	}
	else if(comNumber==1) {
		port = open("/dev/strategyUSB", O_RDWR | O_NOCTTY | O_NDELAY);
		//port = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
	}
	if(port == -1) { perror("openPort: Unable to open"); }
	else { fcntl(port, F_SETFL, 0); }
	return (port);
}

void BarelangStrategy::closePort(){
	close(fd);
}

int BarelangStrategy::setPort(int port){
	tcgetattr(port, &options);
	//cfsetispeed(&options, B9600);
	//cfsetospeed(&options, B9600);
	cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL|CREAD);
	options.c_cflag &= ~CSIZE;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~PARENB;
	options.c_cflag |= CS8;
	options.c_cflag &= ~CRTSCTS;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~OPOST;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 10;
	return tcsetattr (port, TCSANOW, &options);
}

char BarelangStrategy::readSerial(){
	read(fd, &dataSerial, 1);
	return dataSerial;
}

void BarelangStrategy::clearBuffer() {
	for(int i=0;i<BUFLEN_STRATEGY;i++) {
		bufferSerial[i]=0;
	}
}

void BarelangStrategy::readPacketData() {
	lenChar = 0;
	do {
		data = readSerial();
//		printf("%c",data);
		if(data != '\n' && data != NULL) {
//			printf("%c",data);
                        bufferSerial[lenChar] = data;
//			printf("data[%d] = %c\n",lenChar,bufferSerial[lenChar]);
                        lenChar++;
			strncpy(start, bufferSerial, 5);
			start[6] = '\0';
			//printf("start = %c\n",start);

                }
		else{
			lenChar = 0;
		}
	}
	while(data != '\n');
}

void BarelangStrategy::getValue() {
//	printf("length = %d\n",strlen(bufferSerial));
	if((strcmp ("start",start)) == 0) {
		//printf("parsed\n");
		count = 0;
		pch = strtok (bufferSerial,",");

		while (pch!=NULL) {
			if (count == 1)		{ sscanf(pch,"%d",&strategyNumber); }
			else if (count == 2)	{ sscanf(pch,"%d",&killnRun); }
			else if (count == 3)	{ sscanf(pch,"%d",&yaw); }
			else if (count == 4)	{ sscanf(pch,"%d",&pitch); }
			else if (count == 5)	{ sscanf(pch,"%d",&roll); }
			else if (count == 6)	{ count = -1; } //sscanf(pch,"%d",&reset); count = -1; }
			pch = strtok (NULL,",");
			count++;
		}
		//printf("KillProgram : %d\n",killnRun);
	}
//	else printf("error/n");
}

float Zk,Xk,Pk,Kk, Xk_hat, Xk_1=0, Pk_1=1, R_Kal=0.0001, Q=0.00001; //R_Kal=0.0001
int BarelangStrategy::kalmanFilter(int dataIn) {
	Zk=dataIn;
	//Time update
	Xk=Xk_1;
	Pk=Pk_1 + Q;
	//Measurement update
	Kk = Pk / (Pk + R_Kal);
	Xk_hat = Xk + Kk * (Zk - Xk);
	Pk = (1 - Kk) * Pk;
	Xk_1 = Xk_hat;
	Pk_1 = Pk;
	return Xk_hat;
}

int BarelangStrategy::sudutError(int data) {
	if (resetImu > 5) {
		cekWaktuImu(5);
		if (timerImu) {
			if (robotGerak) { offset = offset - 0.05; }
			resetImu = elapsedTimeImu = secondImu = 0;
			timerImu = false;
		} else {
			data = data + offset;
		}
	} else {
		setWaktuImu();
		resetImu++;
	}
	return data;
}

/*double	nK, nT, suTam, suHir;
int BarelangStrategy::sudutTambah(int data) {
	nK = 6;
	nT = 6/90;
	suTam = data * nT;
	suHir = data + suTam;
	return suHir;
}*/
void BarelangStrategy::refresh() {
	resetImu =
	offset =
	elapsedTimeImu =
	secondImu = 0;

	timerImu = false;
}

void BarelangStrategy::checkStrategyControl() {
        fd = openPort(1);
        setPort(fd);
        while(1) {
                readPacketData();
                getValue();
		bearingValue = sudutError(yaw);
		if (killnRun == 1 || reset == 1) {
			refresh();
		}
//		bearingValue = yaw;
		//bearingValue = kalmanFilter(compass);
//		printf("pch : %c\n",pch);
//		printf("data : %c\n",data);
//             	printf("\nStrategy : %d\n",strategyNumber);
//             	printf("KillProgram : %d\n",killnRun);
//             	printf("yaw : %d\n",yaw);
//            	printf("pitch : %d\n",pitch);
//             	printf("roll : %d\n",roll);

//             	printf("bearingValue : %d\n", bearingValue);
//             	printf("offset : %lf\n", offset);
//             	printf("secondImu : %lf\n", secondImu);

//             	printf("\n\n\n");
	}
}
