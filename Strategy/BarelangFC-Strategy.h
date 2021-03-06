#ifndef BARELANGFC_STRATEGY_H_
#define BARELANGFC_STRATEGY_H_

#include <fcntl.h>
#include <termio.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <string.h>

#define BUFLEN_STRATEGY	50

namespace BarelangFC {
	class BarelangStrategy {
		private:
			char bufferSerial[BUFLEN_STRATEGY];
			int fd, port, count, lenChar, compass;
		        char data, dataSerial;
			char start[6];
			char * pch;
			struct termios options;

			int openPort(int comNumber);
			int setPort(int port);
			char readSerial();
			void clearBuffer();
			void readPacketData();
			void getValue();
			int kalmanFilter(int dataIn);
			void setWaktuImu();
			void cekWaktuImu(double detik);
			int sudutError(int data);
			//int sudutTambah(int data);
			void refresh();
		public:
			void closePort();
			void checkStrategyControl();
			int	strategyNumber, bearingValue,
				killnRun, step, robot, reset, yaw, pitch, roll;
			double resetImu, offset;
			bool robotGerak;
	};
}

#endif
