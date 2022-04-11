#ifndef BARELANGFC_VISION_H_
#define BARELANGFC_VISION_H_


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUFLEN 50               //Max length of buffer

namespace BarelangFC {
	class BarelangVision {
		private:
			char * parseCoor;
			int indexing;
			char recvCoor[BUFLEN];
			int iSliderTiltKP, iSliderTiltKI, iSliderTiltKD;
			int iSliderPanKP, iSliderPanKI, iSliderPanKD;
			//FILE *camNotePad;
		public:
			int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D;
			int Goal_X, Goal_Y, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
			int lastBall_X, lastBall_Y;
			double tiltKP, tiltKI, tiltKD;
			double panKP, panKI, panKD;

			void help();
			void initialize();
			void pidTuner();
			void processing();
			void die(char *s);
	};
}

#endif
