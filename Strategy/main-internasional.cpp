//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// Bismillahirrahmanirrahim Tahun ini Juara 1 //////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>

#include "BarelangFC-Vision.h"
#include "BarelangFC-Receiver.h"
#include "BarelangFC-Strategy.h"
#include "BarelangFC-RobotCoordination.h"
#include "BarelangFC-GameControllerClient.h"
#include "BarelangFC-Odometry.h"
#include "BarelangFC-Monitor.h"

//-----------------------config Parameter------------------------
#include "minIni.h"
#define INVALID_VALUE   -1024.0
minIni* ini = new minIni("configure.ini");
//---------------------------------------------------------------

#define	PI	3.1415926535897932384626433832795
#define	PORT	3838
#define	BUFLEN	4096
#define BUFSIZE	1024

#define PORTM	5000	//motion
#define PORTH	5001	//head
#define PORTL   5005    //lokalisasi
#define HOST	"localhost"

//#define SERVER		"192.168.123.30" //IP Address laptop juri
#define SERVER		"192.168.1.1" //IP Address laptop juri
#define TEAM		1 //1 //2 //3 //4 //5 //nomor tim di GameController
#define	robotID		robotNumber //1 //2 //3 //4 //5

#define BARELANG_COLOR	TEAM //inter = TEAM, nasional (0 = cyan, magenta = 1).
#define DROPBALL	128 //128 = inter, 2 = nasional

using namespace BarelangFC;
	pthread_t threadStrategy, threadVision, threadGC_Read, threadGC_Return, Lokalisasi, threadReceiverVision, threadReceiverLocalization, threadPID, threadMonitor;
	BarelangVision kamera;
	BarelangReceiver receler;
	BarelangStrategy strategy;
	RobotCoordination koordinasi;
	GameControllerClient gameController;
	BarelangOdometry odometry;
	BarelangMonitor monitor;

void * sendMonitoring( void * uMonitor) { monitor.sendingMonitor();}
void * updateStrategy( void * uStrategy) { strategy.checkStrategyControl(); }
//void * camera( void * uCamera) { kamera.processing(); }
//void * camera( void * uCamera) { kamera.pidTuner(); }
void * receiverVision( void * uReceiver) { receler.processing_vision(); }
void * receiverLocalization( void * uReceiver) { receler.processing_localization(); }
void * gcReadData(void * argument) { gameController.readGameControllerData(); }
void * gcReturnData(void * argument) { gameController.returnGameControllerData(); }
void runLuaProgram() { system("cd ../Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;");}
//void runLuaProgram() { system("cd ../Player; screen -S dcm lua run_dcm.lua;");}


/////////////////////////////////////////////////////////
////////////////////Fungsi Global////////////////////////
/////////////////////////////////////////////////////////
void	resetCase1();
void	resetCase2();
void	resetCase3();
void	resetCase4();
void	resetCase5();
void	resetCase6();
void	resetCase7();
void	resetCase8();
void	resetOdometry();
void	resetKoordinasiRobotBalik();
void	resetAllLokalisasiVariable();
void	resetAllVariable();
void    refreshMoveLokalisasi();
void	jalanDirection(double, double, double);
void	saveSudutImu();
double 	abs(double);

/////////////////////////////////////////////////////////
///////////////////Variable Global///////////////////////
/////////////////////////////////////////////////////////
int	robotNumber,
	stateCondition = 0,
	firstStateCondition = 0,	//switch strategy

	stateGameController = 0,
	lastStateGameController = 0,
	stateChange = 0,

	kickOff = 0,
	lastKickOff = 0,
	kickOffChange = 0,

	secRemaining = 0,
	lastSecRemaining = 0,
	secRemainingChange = 0,

	state,				//kill n run
	lastState,			//kill n run
	wait = 0,
	lastState2 = 0,			//kill n run
	state2 = 0,			//kill n run
	wait2 = 0,			//kill n run

	delay = 0,			//search goal case 4
	delayWaitBall = 0,		//search ball case 0
	countBearing = 0,		//Imu erorr
	countDribble = 0,		//lama dribble
	tunda = 0,
	tunggu = 0,
	waiting = 0,
	waitTracking = 0,
	reset = 0,
	masha = 0,
	delayTrackBall = 0,
	sebentar = 0,
	matte = 0,
	chotto = 0,

	arahRotate = 0,
	searchTime = 0,
	countPickUp = 0,
	waitingBall = 0,
	kondisi = 1,
	modeKick = 1,

	saveAngle = 0,
	lastDirection = 0,

	countHilang = 0,
	countDapat = 0,

	confirmsBall = 0,
	countTilt = 0,
	sumTilt = 0,
	sumPan = 0,

	Strategi,

	Grid = 0,
	gridX = 0,
	gridY = 0,
	estimasiAngle = 0,

	countInitPos = 0,	//lock initial pos untuk pickup
	countInitPosPen = 0,	//lock initial pos untuk penalti
	robotStatus = 1,	//Robot Aktif

	varCount = 0,		//variable counting untuk case 1001
	countDef = 0,		//variable counting untuk case 130 & 140

	rotateKick = 0;

double	headPan,	//f.Kepala
	headTilt,	//f.Kepala
	posPan,
	posTilt,
	errorPan,	//f.Kepala
	errorTilt,	//f.Kepala
	PPan,		//f.Kepala
	PTilt,		//f.Kepala

	ball_panKP,	//PID trackBall
	ball_panKD,	//PID trackBall
	ball_tiltKP,	//PID trackBall
	ball_tiltKD,	//PID trackBall

	goal_panKP,	//PID trackGoal
	goal_panKD,	//PID trackGoal
	goal_tiltKP,	//PID trackGoal
	goal_tiltKD;	//PID trackGoal

double	ballPositioningSpeed,
	pTiltTendang,
	pPanTendang,
	pTiltOper,
	pPanOper,
	cSekarang,
	cAktif,
	gAktif,
	posTiltLocal,
	posTiltGoal,
	batasTilt,
	erorrXwalk,
	erorrYwalk,
	erorrAwalk,
	jalan,
	lari,
	kejar,
	kejarMid,
	kejarMax,
	tinggiRobot,
	outputSudutY1,
	inputSudutY1,
	outputSudutY2,
	inputSudutY2,
	outputSudutX1,
	inputSudutX1,
	outputSudutX2,
	inputSudutX2,
	frame_X,
	frame_Y,
	rotateGoal_x,
	rotateGoal_y,
	rotateGoal_a,

	sendAngle,
	angle,
	myGoal,
	
	robotPos_X,	//om
	initialPos_X,	//ov
	robotPos_Y,	//om
	initialPos_Y,	//om

	initPos_X,
	initPos_Y,
	
	kurama = 0;

int	sudutTengah,
	sudutKanan,
	sudutKiri,
	tendangJauh,
	tendangDekat,
	tendangSamping;

bool	play = false,
	firstTimes = true,	//first strategy player
	zeroState = false,	//send ball is not found (for coordinations)
	tracked = false,	//search ball
	pickUp = false,		//strategi setelah pick up
	trigger = false,	//motion trigger
	searchRectangle = false,
	startToGoal = false,
	getGoal = false,
	FollowGoal = false,
	tendangLagi = false,

	GoalTracked = false,
	backPosition = false,
	switched = false,
	manual = false,
	dapat = false,
	signIn = false,

	kanan = false,
	kiri = false,

	followSearchAktif = false,
	Activated = false,

	exeCutor = false,

	useVision = true,		//f.setting
	useSocket = true,		//f.setting
	useGameController = true,	//f.setting
	useCoordination = true,		//f.setting
	useLocalization = true,		//f.setting
	useFollowSearchGoal = true,	//f.setting
	useImu = true,			//f.setting
	useDribble = true,		//f.setting
	dribbleOnly = true,		//f.setting
	useSearchGoal = true,		//f.setting
	useAutoDirection = true,	//f.setting
	useSideKick = true,		//f.setting
	useLastDirection = true,	//f.setting
	useNearFollowSearchGoal = true,	//f.setting
	useUpdateCoordinate = true,	//f.setting
	usePenaltyStrategy = true;		//f.setting

//Socket LUA head&motion
int	sockmotion;
int	sockhead;
struct	sockaddr_in addrmotion;
struct	sockaddr_in addrhead;
struct	hostent *localserver;
char	dataMotion[20];
char	dataHead[20];
char	line[10];

//Socket Lokalisasi
int     socklokalisasi;
struct  sockaddr_in addrlokalisasi;
char    dataLokalisasi[30];
char    param[15];

///////////////////////////////////////////////////////////////////
///////////////////////open port LUA///////////////////////////////
///////////////////////////////////////////////////////////////////
void initSendDataMotion() {
	sockmotion = socket(AF_INET, SOCK_DGRAM, 0);
	localserver = gethostbyname(HOST);
	bzero((char *) &addrmotion, sizeof(addrmotion));
	addrmotion.sin_family = AF_INET;
	addrmotion.sin_port = htons(PORTM);
}

void initSendDataHead() {
	sockhead = socket(AF_INET, SOCK_DGRAM, 0);
	localserver = gethostbyname(HOST);
	bzero((char *) &addrhead, sizeof(addrhead));
	addrhead.sin_family = AF_INET;
	addrhead.sin_port = htons(PORTH);
}


///////////////////////////////////////////////////////////////////
////////////////////open port Lokalisasi///////////////////////////
///////////////////////////////////////////////////////////////////
void initSendDataLokalisasi() {
        socklokalisasi = socket(AF_INET, SOCK_DGRAM, 0);
        localserver = gethostbyname(HOST);
        bzero((char *) &addrlokalisasi, sizeof(addrlokalisasi));
        addrlokalisasi.sin_family = AF_INET;
        addrlokalisasi.sin_port = htons(PORTL);
}


// Function To Set Timer =====================================================================
struct timeval	t1,
		t2;
double	elapsedTime,
	second;
bool	timer = false;
void setWaktu() {
	elapsedTime	=
	second		= 0;
	timer		= false;

	gettimeofday(&t1, NULL);
}
// Function For Check Timer
void cekWaktu(double detik) {
	gettimeofday(&t2, NULL);

	// compute and print the elapsed time in millisec
	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
	elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
	second = elapsedTime / 1000.0;
	//printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

	if (second >= detik) {
		timer = true;
	} else {
		timer = false;
	}
}

// Function To Set Timer For Lokalisasi ======================================================
struct timeval	time1,
		time2;
double	elapsedTime2,
	second2;
bool	timer2 = false;
void setWaktuLokalisasi() {
	elapsedTime2	=
	second2		= 0;
	timer2		= false;

	gettimeofday(&time1, NULL);
}
// Function For Check Timer
void cekWaktuLokalisasi(double detik) {
	gettimeofday(&time2, NULL);

	// compute and print the elapsed time in millisec
	elapsedTime2 = (time2.tv_sec - time1.tv_sec) * 1000.0;
	elapsedTime2 += (time2.tv_usec - time1.tv_usec) / 1000.0;
	second2 = elapsedTime2 / 1000.0;
	//printf ("  waktu berlangsung = %.f detik \n\n\n\n", second2);

	if (second2 >= detik) {
		timer2 = true;
	} else {
		timer2 = false;
	}
}


// Absolute ==================================================================================
double abs(double x) {
	if (x < 0) { x = -x; }
	return x;
}

// Sensor Accelero, Gyro, Angle Value ========================================================
double	accrX, accrY, accrZ,
	gyroX, gyroY, gyroZ,
	angleX, angleY, angleZ,
	myAccrX, myAccrY;
bool	robotJatuh = false,
	robotBergerak = false;
int	countParse = 0,
	countMove = 0;
void getSensor() {
	char line[100];
	char * spr;

	FILE *outputfp;
	outputfp = fopen("../Player/SensorNote", "r");
	fscanf(outputfp, "%s", &line);
	countParse = 0;
	spr = strtok (line, ";");
	while (spr != NULL) {
		if (countParse == 0)	  { sscanf(spr, "%lf", &accrX); } //x					#dpn-blkng
		else if (countParse == 1) { sscanf(spr, "%lf", &accrY); } //y					#kiri-kanan
		else if (countParse == 2) { sscanf(spr, "%lf", &accrZ); } //z
		else if (countParse == 3) { sscanf(spr, "%lf", &gyroX); } //roll				#kiri-kanan
		else if (countParse == 4) { sscanf(spr, "%lf", &gyroY); } //pitch				#dpn-blkng
		else if (countParse == 5) { sscanf(spr, "%lf", &gyroZ); } //yaw
		else if (countParse == 6) { sscanf(spr, "%lf", &angleX); } //
		else if (countParse == 7) { sscanf(spr, "%lf", &angleY); } //
		else if (countParse == 8) { sscanf(spr, "%lf", &angleZ); countParse = -1; } //
		spr = strtok (NULL,";");
		countParse++;
	}
	fclose(outputfp);
	//printf("  accr(%.2lf, %.2lf, %.2lf)", accrX, accrY, accrZ);
	//printf("  gyro(%.2lf, %.2lf, %.2lf)", gyroX, gyroY, gyroZ);
	//printf("  angle(%.lf, %.lf, %.lf)", angleX, angleY, angleZ);

	// menunjukkan bahwa robot sedang dalam kondisi jatuh -> back to case 0
	if ((accrX >= -0.5) && (accrX <= 0.9) && (accrY >= -0.9) && (accrY <= 0.9)) { //printf("  robot berdiri...................\n\n");
		robotJatuh = false;
	} else { //printf("  ...................robot jatuh.\n\n");
		robotJatuh = true;
	}

	// menunjukkan bahwa robot sudah berdiri dan bergerak -> syarat trajectory (mapping)
	//if (gyroX > 4 || gyroX < -4) { //printf("  robot bergerak............\n");
	if (gyroX != 0) { //printf("  robot bergerak............\n");
		countMove = 0;
		robotBergerak = true;
	} else { //printf("  ............robot TDK bergerak\n");
		if (countMove > 10) {
			robotBergerak = false;
		} else {
			countMove++;
		}
	}
}

int	acclrX;
bool	robotFall = false;
void getImuSensor() {
	acclrX = strategy.pitch;
	if (acclrX >= -50 && acclrX <= 50) { //printf("  robot berdiri...................\n\n");
		robotFall = false;
	} else { //printf("  ...................robot jatuh.\n\n");
		robotFall = true;
	}
}

// Sudut 20 Servo ===========================================================================
double	head1, head2,
	Larm1, Larm2, Larm3,
	Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6,
	Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6,
	Rarm1, Rarm2, Rarm3;
int	countParses = 0;
void getServPos() {
	char line[200];
	char * sprs;

	FILE *outputfp;
	outputfp = fopen("../Player/PosNote", "r");
	fscanf(outputfp, "%s", &line);
	countParses = 0;
	sprs = strtok (line, ";");
	while (sprs != NULL) {
		if (countParses == 0)	  { sscanf(sprs, "%lf", &head1); }			//ID=19
		else if (countParses == 1) { sscanf(sprs, "%lf", &head2); }			//ID=20
		else if (countParses == 2) { sscanf(sprs, "%lf", &Larm1); }			//ID=2
		else if (countParses == 3) { sscanf(sprs, "%lf", &Larm2); }			//ID=4
		else if (countParses == 4) { sscanf(sprs, "%lf", &Larm3); }			//ID=6
		else if (countParses == 5) { sscanf(sprs, "%lf", &Lleg1); odometry.L_leg1 = Lleg1;}			//ID=8
		else if (countParses == 6) { sscanf(sprs, "%lf", &Lleg2); odometry.L_leg2 = Lleg2;}			//ID=10
		else if (countParses == 7) { sscanf(sprs, "%lf", &Lleg3); odometry.L_leg3 = Lleg3;}			//ID=12
		else if (countParses == 8) { sscanf(sprs, "%lf", &Lleg4); odometry.L_leg4 = Lleg4;}			//ID=14
		else if (countParses == 9) { sscanf(sprs, "%lf", &Lleg5); odometry.L_leg5 = Lleg5;}			//ID=16
		else if (countParses == 10) { sscanf(sprs, "%lf", &Lleg6); odometry.L_leg6 = Lleg6;}			//ID=18
		else if (countParses == 11) { sscanf(sprs, "%lf", &Rleg1); odometry.R_leg1 = Rleg1;}			//ID=7
		else if (countParses == 12) { sscanf(sprs, "%lf", &Rleg2); odometry.R_leg2 = Rleg2;}			//ID=9
		else if (countParses == 13) { sscanf(sprs, "%lf", &Rleg3); odometry.R_leg3 = Rleg3;}			//ID=11
		else if (countParses == 14) { sscanf(sprs, "%lf", &Rleg4); odometry.R_leg4 = Rleg4;}			//ID=13
		else if (countParses == 15) { sscanf(sprs, "%lf", &Rleg5); odometry.R_leg5 = Rleg5;}			//ID=15
		else if (countParses == 16) { sscanf(sprs, "%lf", &Rleg6); odometry.R_leg6 = Rleg6;}			//ID=17
		else if (countParses == 17) { sscanf(sprs, "%lf", &Rarm1); }			//ID=1
		else if (countParses == 18) { sscanf(sprs, "%lf", &Rarm2); }			//ID=3
		else if (countParses == 19) { sscanf(sprs, "%lf", &Rarm3); countParses = -1; }	//ID=5
		sprs = strtok (NULL,";");
		countParses++;
	}
	fclose(outputfp);
	//printf("  head(%.lf, %.lf)", head1, head2);
	//printf("  Larm(%.lf, %.lf, %.lf)", Larm1, Larm2, Larm3);
	//printf("  Rarm(%.lf, %.lf, %.lf)", Rarm1, Rarm2, Rarm3);
	//printf("  Lleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
	//printf("  Rleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
}


// Mapping ================================================================================
/*double  chuis,
	coorXx, coorXy, coorYx, coorYy,
	positionXnow, positionYnow;

void setPosition (double x, double y) {
	positionXnow = x;
	positionYnow = y;
}

void mapping(double x, double y) {
	sudut();
	getSensor();

	// bagian X
	coorXx = (x * (cos((abs(angle)) * PI / 180)));
	coorXy = (x * (sin(angle * PI / 180)));

	// bagian Y
	if (angle <= 180 && angle >= -90) { //angle <= 180 &&
		coorYx = (y * (cos((abs(angle - 90)) * PI / 180)));
		coorYy = (y * (sin((angle - 90) * PI / 180)));
	} else {
		coorYx = (y * (cos((abs(angle + 270)) * PI / 180)));
		coorYy = (y * (sin((angle + 270) * PI / 180)));
	}

	if (robotJatuh == false && robotBergerak == true) {
		positionXnow = positionXnow + coorXx + coorYx;
		positionYnow = positionYnow + coorYy + coorXy;
	} //printf("  Coor(%.2lf, %.2lf)", positionXnow, positionYnow);
}*/

// Walk Controller ===========================================================================
double  walkX = 0.0,
        walkY = 0.0,
        walkA = 0.0;
double Walk(double x, double y, double a) {

	char line[50];

	walkX = x;
	walkY = y;
        walkA = a;

	odometry.RobotWalk_X = walkX; //+ erorrXwalk;
	odometry.RobotWalk_Y = walkY; //+ erorrYwalk;
	odometry.RobotWalk_A = walkA; //+ erorrAwalk;

	if (robotID != 2 || robotID != 5) {
		if (odometry.RobotWalk_X >= 0.08) { odometry.RobotWalk_X = 0.08; }
		else if (odometry.RobotWalk_X <= -0.03) { odometry.RobotWalk_X = -0.03; }

		if (odometry.RobotWalk_Y >= 0.03) { odometry.RobotWalk_Y = 0.03; }
		else if (odometry.RobotWalk_Y <= -0.03) { odometry.RobotWalk_Y = -0.03; }

		if (odometry.RobotWalk_A >= 0.4) { odometry.RobotWalk_A = 0.4; }
		else if (odometry.RobotWalk_A <= -0.4) { odometry.RobotWalk_A = -0.4; }
	} else {
		if (odometry.RobotWalk_X >= 0.06) { odometry.RobotWalk_X = 0.06; }
		else if (odometry.RobotWalk_X <= -0.03) { odometry.RobotWalk_X = -0.03; }

		if (odometry.RobotWalk_Y >= 0.03) { odometry.RobotWalk_Y = 0.03; }
		else if (odometry.RobotWalk_Y <= -0.03) { odometry.RobotWalk_Y = -0.03; }

		if (odometry.RobotWalk_A >= 0.4) { odometry.RobotWalk_A = 0.4; }
		else if (odometry.RobotWalk_A <= -0.4) { odometry.RobotWalk_A = -0.4; }
	}

	if (useSocket) { // Socket
		strcpy(line, "walk");
		sprintf(dataMotion, "%s,%f,%f,%f", line, x+erorrXwalk, y+erorrYwalk, a+erorrAwalk);
		sendto(sockmotion, dataMotion, strlen(dataMotion), 0,(struct sockaddr *) &addrmotion, sizeof(addrmotion));
		//printf("  data walk = %s\n", dataMotion);
	} else { // NotePad
		FILE *outputfp;
		strcpy(line, "walk");
		outputfp = fopen("../Player/WalkNote", "wb");
		fprintf(outputfp, "%s,%.2lf,%.2lf,%.2lf", &line[0], x+erorrXwalk, y, a);//setting jalan ditempat default
		fclose(outputfp);
		//printf("walk : %g,%g,%g\n",x,y,a);
	}

	//if (useMapping) {
	//	mapping((x + errorWalk), y); //(x + (errorWalk * -1))
	//}

	return (walkX,walkY,walkA);
}


// Step Count ================================================================================
/*int	step = 0,
	lastStep = 0,
	sumStep = 0,
	lastPressure = 0;
void steps() {
	//if (lastPressure != strategy.step) {
	//	if (lastPressure < strategy.step) {
	//		step = 1;
	//	} else {
	//		step = 0;
	//	} lastPressure = strategy.step;
	//}

	if (strategy.step > -900) {
		step = 1;
	} else {
		step = 0;
	} //printf("  weight = %d, step = %d \n", strategy.step, step);

	if (lastStep != step) {
		if (step == 1) {
			sumStep++;
		} lastStep = step;
		//printf("  sumStep = %d\n\n", sumStep);
	}
}

// prediksi langkah ==========================================================================
double	dist = 0,
	hituh = 0,
	ms = 0;
bool	ferse = true;
void predictDist(double jarak, double kecepatan) { //jarak max = 9 m, kecepatan max = 0.25 m/s
	if (ferse) { //printf("_\n");
		if (robotBergerak) {
			if (hituh >= 30) {
				ferse = false;
			} else {
				setWaktu();
			} hituh++;
		}
		Walk(0.0, 0.0, 0.0);
	} else { //printf("+\n");
		cekWaktu(1);
		ms = (37292*(pow(kecepatan,4))) - (8916.2*(pow(kecepatan,3))) + (652.03*(pow(kecepatan,2))) - (13.565*kecepatan) + 0.1466;
		dist = ms * second;
		dist *= 100;
		if (dist >= jarak) {
			//motion("0");
			hituh = 0;
			Walk(0.0, 0.0, 0.0);
		} else {
			//motion("9");
			//ms = (-2.1419 * (pow(kecepatan,3))) + (0.1869 * (pow(kecepatan,2))) + (0.3246 * kecepatan) + 0.0006;
			Walk(kecepatan, 0.0, 0.0);
			printf("dist = %.2f, kec = %.2f, sec = %.2f\n", dist, kecepatan, second);
		}
	}
}*/

// Robot Movement ============================================================================
void motion(char line[2]) {
	//Tendang Jauh Kiri		= 1
	//Tendang Jauh Kanan		= 2

	//Tendang Pelan Kiri		= 3
	//Tendang Pelan Kanan		= 4

	//Tendang Ke Samping Kiri	= 5
	//Tendang Ke Samping Kanan	= 6

	//Tendang WalkKick Kiri		= t
	//Tendang WalkKick Kanan	= y

	//Duduk				= 7
	//Berdiri			= 8
	//Play				= 9
	//Stop				= 0

	if (useSocket) { // Socket
		char awalan[50];
		strcpy(awalan, "motion");
		sprintf(dataMotion, "%s,%s",awalan, line);
		sendto(sockmotion, dataMotion, strlen(dataMotion), 0, (struct sockaddr *) &addrmotion, sizeof(addrmotion));
		//printf("  data motion = %s\n", dataMotion);
	} else { // NotePad
		FILE *outputfp;
		outputfp = fopen("../Player/WalkNote", "wb");
		fprintf(outputfp, "%s", &line[0]);
		fclose(outputfp);
	}

	// untuk kondisi offset imu
	if (line == "9") {
		strategy.robotGerak = true;
	} else if (line == "0") {
		strategy.robotGerak = false;
	}
}

// Checking Lost Ball ========================================================================
int	countBallLost  = 0,
	countBallFound = 0,
	returnBallVal;
int ballLost(int threshold) {
	if (useVision) {
		if (receler.Ball_X == -1 && receler.Ball_Y == -1) {
			countBallFound = 0;
			countBallLost++;
			if (countBallLost >= threshold) {
				returnBallVal = 1;
			}
		} else {
			countBallLost = 0;
			countBallFound++;
			if (countBallFound > 1) {
				returnBallVal = 0;
			}
		}
	} else {
		countBallFound = 0;
		countBallLost++;
		if (countBallLost >= threshold) {
			returnBallVal = 1;
		}
	} return returnBallVal;
}

// Head Movement =============================================================================
void headMove(double pan,double tilt) {
	if (useSocket) { // Socket
		sprintf(dataHead, "%.2f,%.2f", pan, tilt);
		sendto(sockhead, dataHead, strlen(dataHead), 0,(struct sockaddr *) &addrhead, sizeof(addrhead));
		//printf("  data head = %s\n", dataHead);
	} else { // NotePad
		FILE *fp, *outputfp;
		outputfp = fopen("../Player/HeadNote", "wb");
		fprintf(outputfp, "%.2lf,%.2lf", pan,tilt);
		fclose(outputfp);
	}

	headPan = posPan = pan;
	headTilt = posTilt = tilt;
	//printf("headPan = %.2f \t headTilt = %.2f\n", pan, tilt);
}

// Search ball ===============================================================================
double	tiltRate = -0.05,//-0.03,//-0.075
	panRate  = -0.05,//-0.04,//-0.075

	tiltRate1 = -0.06,
	panRate1  = -0.06,

	tiltRate2 = -0.08,
	panRate2  = -0.08,

	tiltRate3 = -0.1,
	panRate3  = -0.1,

	tiltRate4 = -0.07,
	panRate4  = -0.07,

	searchKe = 0,

	batasKanan = -1.6,
	batasKiri  =  1.6,
	batasAtas  = -2.0,
	batasBawah = -0.6;

void tiltSearchBall(double tempPosPan) { //printf("  tiltSearchBall\n\n");
	posPan = tempPosPan;
	posTilt += tiltRate1;

	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate1 *= -1;
	}

	if (headTilt <= (-1.2 + tiltRate) && headTilt >= (-1.2 - tiltRate)) {
		//searchKe += 0.5;
	}

	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

void panSearchBall(double tempPosTilt) { //printf("  panSearchBall\n\n");
	posTilt = tempPosTilt;
	posPan += panRate2;

	if (posPan <= batasKanan || posPan >= batasKiri) {
		panRate2 *= -1;
	}

	if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate)) {
		//searchKe += 0.5;
	}

	if	(posPan >= batasKiri)  { posPan = batasKiri; }
	else if	(posPan <= batasKanan) { posPan = batasKanan; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

int	i = 1,
	panKe = 2,
	tiltKe = 1;
double	panSearch[5] = {1.6, 0.8, 0.0, -0.8, -1.6},
	//tiltSearch1[3] = {-0.6, -1.2, -1.8},
	tiltSearch1[3] = {-0.6, -1.4, -1.8},
	tiltSearch2[2] = {-0.6, -1.4};
void SearchBall(int mode) { //printf("  normalSearchBall\n\n");
	if (mode == 1) { // (atas-bawah)
		posTilt += tiltRate;
		if (posTilt <= batasAtas || posTilt >= batasBawah) {
			if (panKe == 2) { searchKe += 1; }

			tiltRate *= -1;
			panKe += i;
			if (panKe >=  4 || panKe <= 0) {
				i = -i;
			}
		} posPan = panSearch[panKe]; //printf("count pan = %d\n", panKe);

	} else if (mode == 2) { // (kiri-kanan)
		posPan += panRate4;
		if (posPan <= batasKanan || posPan >= batasKiri) {
			if (tiltKe == 1) { searchKe += 1; }
			panRate4 *= -1;
			tiltKe += i;

			if (tiltKe >=  2 || tiltKe <= 0) {
				i = -i;
			}
		} posTilt = tiltSearch1[tiltKe]; //printf("count tilt = %d\n", tiltKe);

	} else if (mode == 3) { // muter-muter
		if (countTilt == 0) {
			posPan += panRate3;
		} else if (countTilt == 1) {
			posPan += panRate2;
		}
		//posPan += panRate;
		if(posPan <= batasKanan || posPan >= batasKiri) {
			panRate2 *= -1;
			panRate3 *= -1;
			//panRate *= -1;
			countTilt++;
			if (countTilt > 1) countTilt = 0;
		}
		posTilt = tiltSearch2[countTilt]; //printf("count tilt = %d\n", countTilt);
	}

	if	(posPan >= batasKiri)   { posPan = batasKiri; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

int	sabar = 0;
void threeSearchBall() {
	if (sabar > 7) {
		if (tiltKe == 0) {
			posPan += panRate2;
		} else if (tiltKe == 1) {
			posPan += panRate2; //panRate2;
		} else if (tiltKe == 2) {
			posPan += panRate2; //panRate1;
		}

		if (posPan <= batasKanan || posPan >= batasKiri) {
			if (tiltKe == 2 && posPan <= -1.5) { searchKe += 1; }

			//panRate1 *= -1;
			panRate2 *= -1;
			//panRate3 *= -1;

			tiltKe += i;
			if (tiltKe >=  2 || tiltKe <= 0) {
				i = -i;
			}
		} posTilt = tiltSearch1[tiltKe]; //printf("count tilt = %d\n", tiltKe);
	} else {
		posTilt = batasBawah;
		posPan = batasKiri;
		//panRate1  = -0.06;
		panRate2  = -0.08;
		//panRate3  = -0.1;
		tiltKe = 0;
		i = 1;
		sabar++;
	}

	if	(posPan >= batasKiri)   { posPan = batasKiri; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

void rightSearchBall() { //printf("  rightSearchBall\n\n");
	posTilt += tiltRate;
	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate *= -1;
		posPan += panRate;
		if (posPan >= 0.0 || posPan <= batasKanan) {
			panRate *= -1;
		}
	}

	if	(posPan >= 0.0)   { posPan = 0.0; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt);
}

void leftSearchBall() { //printf("  leftSearchBall\n\n");
	posTilt += tiltRate;
	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate *= -1;
		posPan += panRate;
		if (posPan >= batasKiri || posPan <= 0.0) {
			panRate *= -1;
		}
	}

	if	(posPan <= 0.0)  { posPan = 0.0; }
	else if	(posPan >= batasKiri)   { posPan = batasKiri; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt);
}

double	invPan;
void searchBallPan(double uPan, double uTilt) {
	posTilt = uTilt;
	invPan = uPan*-1;

	posPan += panRate;

	if	(posPan <= invPan) { posPan = invPan; panRate *= -1; }
	else if	(posPan >= uPan  ) { posPan = uPan;   panRate *= -1; }

	headMove(posPan, posTilt);
}

bool neckX;
void searchBallRectang(double atas, double kanan, double bawah, double kiri) {
	if (neckX) {
		posPan += panRate2;
		if (posPan >= kiri || posPan <= kanan) {
			panRate2 *= -1;
			neckX = false;
		}
	} else {
	        posTilt += tiltRate1;
		if (posTilt <= atas || posTilt >= bawah) {
			tiltRate1 *= -1;
			neckX = true;
		}
	}

	if	(posPan >= kiri)	{ posPan = kiri; }
	else if (posPan <= kanan)	{ posPan = kanan; }
	if	(posTilt <= atas)	{ posTilt = atas; }
	else if (posTilt >= bawah)	{ posTilt = bawah; }

	headMove(posPan, posTilt); //printf("pan = %f, tilt = %f\n",posPan,posTilt);
}

bool Move;
void searchBallRectang2(double atas, double kanan, double bawah, double kiri) {
	if (Move) {
		posPan += panRate3;
		if (posPan >= kiri || posPan <= kanan) {
			panRate3 *= -1;
			Move = false;
		}
	} else {
	        posTilt += tiltRate3;
		if (posTilt <= atas || posTilt >= bawah) {
			tiltRate3 *= -1;
			Move = true;
		}
	}

	if	(posPan >= kiri)	{ posPan = kiri; }
	else if (posPan <= kanan)	{ posPan = kanan; }
	if	(posTilt <= atas)	{ posTilt = atas; }
	else if (posTilt >= bawah)	{ posTilt = bawah; }

	headMove(posPan, posTilt); //printf("pan = %f, tilt = %f\n",posPan,posTilt);
}

double	ballPan = 0,
	ballTilt = 0;
void saveBallLocation() {
//	trackBall();
	ballPan = posPan;
	ballTilt = posTilt;
}

void loadBallLocation(double tilt) {
	//posPan = 0.0;
	//posTilt = -0.8;
	posPan = ballPan;
	posTilt = ballTilt + tilt;
	headMove(posPan, posTilt);
}

int	semeh;
int koordinasiJarak() {
	semeh = (int)(headTilt*-100);
	return semeh;
}

// Ball Tracking =============================================================================
double	intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
	PPanB = 0, IPanB = 0, DPanB = 0,
	intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
	PTiltB = 0, ITiltB = 0, DTiltB = 0,
	dtB = 0.04;
double	B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
	pOffsetB, iOffsetB, dOffsetB,
	errorPanBRad, errorTiltBRad;
void trackBall() {
	if (useVision) {
		if (receler.Ball_X != -1 && receler.Ball_Y != -1) { //printf("Tracking");
			//mode 1 ######################################################################
				/*// PID pan ==========================================================
				errorPanB  = (double)Ball_x - (frame_X / 2);//160
				PPanB  = errorPanB  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

				intPanB += errorPanB * dtB;
				IPanB = intPanB * 0.0;

				dervPanB = (errorPanB - preErrPanB) / dtB;
				DPanB = dervPanB * 0.00001;

				preErrPanB = errorPanB;

				//posPan += PPanB*-1; //dikali -1 kalau receler terbalik dalam pemasangan
				posPan += (PPanB + IPanB + DPanB) * -1;


				// PID tilt ==========================================================
				errorTiltB = (double)Ball_y - (frame_Y / 2);//120
				PTiltB = errorTiltB * 0.00010; // Tune in Kp Tilt 0.00030

				intTiltB += errorTiltB * dtB;
				ITiltB = intTiltB * 0.0;

				dervTiltB = (errorTiltB - preErrTiltB) / dtB;
				DTiltB = dervTiltB * 0.00001;

				preErrTiltB = errorTiltB;

				//posTilt += PTiltB;
				posTilt += (PTiltB + ITiltB + DTiltB);*/

			//mode 2 ######################################################################
				errorPanB  = (double)receler.Ball_X - (frame_X / 2);//160
				errorTiltB = (double)receler.Ball_Y - (frame_Y / 2);//120
				errorPanB *= -1;
				errorTiltB *= -1;
				errorPanB *= (77.32 / (double)frame_X); // pixel per angle
				errorTiltB *= (61.93 / (double)frame_Y); // pixel per angle

				errorPanBRad = (errorPanB * PI)/ 180;
				errorTiltBRad = (errorTiltB * PI) /180;
				//printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
				//printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
				//printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

				B_Pan_err_diff = errorPanBRad - B_Pan_err;
				B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

				// PID pan ==========================================================
				//PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				PPanB  = B_Pan_err  * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				intPanB += B_Pan_err * dtB;
				IPanB = intPanB * 0.0;
				dervPanB = B_Pan_err_diff / dtB;
				//DPanB = dervPanB * kamera.panKD;
				DPanB = dervPanB * ball_panKD;
				B_Pan_err = errorPanBRad;
				posPan += (PPanB + IPanB + DPanB);

				// PID tilt ==========================================================
				//PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
				PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

				intTiltB += B_Tilt_err * dtB;
				ITiltB = intTiltB * 0.0;

				dervTiltB = B_Tilt_err_diff / dtB;
				//DTiltB = dervTiltB * kamera.tiltKD;
				DTiltB = dervTiltB * ball_tiltKD;

				preErrTiltB = errorTiltB;
				B_Tilt_err = errorTiltBRad;
				posTilt += (PTiltB + ITiltB + DTiltB) * -1;


			if      (posPan  >=  1.6)  { posPan  =  1.6; }
			else if (posPan  <= -1.6)  { posPan  = -1.6; }
			if      (posTilt <= -2.0)  { posTilt = -2.0; }
			else if (posTilt >= -0.4)  { posTilt = -0.4; }

			headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

			koordinasiJarak();			
			saveBallLocation();			
		}
	}
}

// Body Tracking Ball ========================================================================
double	errorBodyPosition,
	bodyP_Controller;
int	bodyTrue = 0,
	delayTrue = 0;
int bodyTrackingBall(int threshold) {
	errorBodyPosition = 0 - headPan;
	bodyP_Controller = errorBodyPosition * -0.5;//-0.5

	if (ballLost(20)) {
		Walk(0.0, 0.0, 0.0);
		bodyP_Controller = bodyTrue = delayTrue = 0;
	} else {
		if (errorBodyPosition >= -0.3 && errorBodyPosition <= 0.3) {//untuk hasil hadap 0.8
			//motion("0");
			Walk(0.0, 0.0, 0.0);
			delayTrue++;
		} else {
			trackBall();
			//motion("9");

			if (bodyP_Controller < 0) { bodyP_Controller = -0.2;  } //kanan 0.15
			else { bodyP_Controller = 0.2; } //kiri 0.15

			bodyTrue = delayTrue = 0;
			Walk(0.0, 0.0, bodyP_Controller);
		}

		if (delayTrue >= threshold) {
			bodyTrue = 1;
		} else {
			bodyTrue = 0;
		}
	} //printf("Body Error = %.2f\t Body P Controller = %.2f\n",errorBodyPosition,bodyP_Controller);
	return bodyTrue;
}

// Hitung Jarak Bola berdasarkan headTilt ==============================================================
double	alphaY,		//hasil derajat ketika headTilt
	betaY,
	inputY,		//nilai realtime headTilt

	alphaX,		//hasil derajat ketika headPan
	betaX,
	inputX,		//nilai realtime headPan

	jarakBola_Y,	//hasil jarak(cm) dari kalkulasi headTilt
	jarakBola_X,	//hasil jarak(cm) dari kalkulasi headPan
	jarakBola;

void kalkulasiJarakBola() {
	inputY	= headTilt;
	inputX	= headPan;

	//alphaY = -57.29 * headTilt; //(metode 1)
	alphaY = outputSudutY1 + ((outputSudutY2 - outputSudutY1) / (inputSudutY2 - inputSudutY1)) * (inputY - inputSudutY1);  //(metode 2) //printf("  alphaY = %.2f,", alphaY);
	betaY = 180 - (90 + alphaY);  //printf("  betaY = %.2f,", betaY);

	//alphaX = -57.29 * headPan; //(metode 1)
	alphaX = outputSudutX1 + ((outputSudutX2 - outputSudutX1) / (inputSudutX2 - inputSudutX1)) * (inputX - inputSudutX1);  //(metode 2) //printf("  alphaX = %.2f,", alphaX);
	betaX = 180 - (90 + alphaX);  //printf("  betaX = %.2f,", betaX);

	//sin & cos dalam c++ adalah radian, oleh karena itu harus : sin( ... * PI / 180)
	jarakBola_Y = (tinggiRobot * sin(alphaY * PI / 180)) / sin(betaY * PI / 180);  //printf("  jarakBola_Y = %.f,", jarakBola_Y);
	jarakBola_X = (jarakBola_Y * sin(alphaX * PI / 180)) / sin(betaX * PI / 180);  //printf("  jarakBola_X = %.f\n\n\n\n", jarakBola_X);
	jarakBola = sqrt((jarakBola_Y * jarakBola_Y) + (jarakBola_X * jarakBola_X));

	//regresi (metode 3)
	//jarakBola_Y = (-933.9*(pow(posTilt,5))) + (-5340.8*(pow(posTilt,4))) + (-12018*(pow(posTilt,3))) + (-13183*(pow(posTilt,2))) + (-7050.2*posTilt) - 1454.3;
}

// Untuk Kalkulasi Posisi P1
double 	P1_X, P1_Y;
void hitungKoordinatBolaP1() {
	//mode1--------------
		kalkulasiJarakBola();
		P1_X = jarakBola_X;
		P1_Y = jarakBola_Y;
	//mode2--------------
		//P1_Y = receler.Ball_J;
	//printf("  P1_X = %.2f,  P1_Y = %.2f,", P1_X, P1_Y);
}

// Untuk Kalkulasi Posisi P2
double 	P2_X, P2_Y;
void hitungKoordinatBolaP2() {
	//mode1--------------
		kalkulasiJarakBola();
		P2_X = jarakBola_X;
		P2_Y = jarakBola_Y;
	//mode2--------------
		//P2_Y = receler.Ball_J;
	//printf("  P2_X = %.2f,  P2_Y = %.2f,", P2_X, P2_Y);
}

// Untuk penyebut dan pembilang gradient
double 	deltaY, deltaX;
void hitungDeltaY() {
	deltaY = P2_Y - P1_Y; //mendekat positif
	//deltaY = P1_Y - P2_Y; //mendekat positif
	//printf("  deltaY = %.2f,", deltaY);
}

void hitungDeltaX() {
	deltaX = P2_X - P1_X; //mendekat negatif
	//deltaX = P1_X - P2_X; //mendekat positif
	//printf("  deltaX = %.2f,\n\n", deltaX);
}

// Untuk Gradient
double 	gradient;
void hitungGradient() {
	gradient = deltaY / deltaX;
	//printf("  gradient = %.2f,", gradient);
}

int	cntOke1,
	cntOke2,
	cntUlang,
	kondisiBola = 0;
bool	oke     = true,
	ulang   = false;
void hitungGerakBola() { // Bagian pengecekan pergerakan bola
	if (receler.Ball_X == -1 && receler.Ball_Y == -1) { //bola hilang
		deltaY		=
		deltaX		=
		jarakBola_X	=
		jarakBola_Y	= 0;
	} //else {
	//	trackBall();
	//}

	if (ulang) { //printf("ulang \n");
		cntOke1 =
		cntOke2 = 0;
		if (cntUlang > 5) {
			ulang = false;
			oke = true;
		} else {
			deltaY		=
			deltaX		=
			jarakBola_X	=
			jarakBola_Y	= 0;

			cntUlang++;
		}
	}

	if (oke) { //printf("oke \n");
		cntUlang = 0;
		if (cntOke1 > 5) {
			hitungKoordinatBolaP2();

			hitungDeltaY();
			//hitungDeltaX();
			//hitungGradient();

			if (cntOke2 > 10) {
				oke = false;
				ulang = true;
			} else {
				cntOke2++;
			}
		} else {
			hitungKoordinatBolaP1();
			cntOke1++;
		}
	}

	//printf("  deltaY = %.f,", deltaY);
	if (deltaY >= 5) { //0.5 //7.0
		//printf("  deltaY = %.f,", deltaY);
		//printf("  Bola Menjauh\n");
		kondisiBola = 1;
	}
	else if(deltaY <= -5) { //-2 //-1.4
		//printf("  deltaY = %.f,", deltaY);
		//printf("  Bola Mendekat\n");
		kondisiBola = -1;
		//if (deltaY <= -2 && headTilt >= -1.2) {
		//	printf("  BANTING BADAN........!!!!!!!\n\n\n\n\n\n\n");
		//}
	} else {
		//printf("  Bola Diam");
		kondisiBola = 0;
		//if (calculateErrorStaticBallPosition(10)) {//ball is static 10
		//	if (headTilt <= -1.4) { //ball is far
		//		printf("  Bola Masih Jauh\n\n\n\n\n\n\n");
		//		hitungDeltaY();
		//	} else { //ball is near
		//		printf("  Bola DEKAT Ehhhh....!!!!\n\n\n\n\n\n\n");
		//	}
		//}
	}
}

// Follow Ball ===============================================================================
int	countReadyKick;
double	SetPointPan = 0,
	SetPointTilt = -0.8,//-0.08
	errorfPan,
	errorfTilt,
	PyMove = 0,
	PxMove = 0,
	PaMove = 0;
void followBall(int mode){ //0 normal, 1 sambil belok
	trackBall();

	if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
	else if	(posTilt < -2.0) { posTilt = -2.0; }

	errorfPan  = posPan - SetPointPan;
	errorfTilt = posTilt - SetPointTilt;

	if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && receler.Ball_X != -1 && receler.Ball_Y != -1) { //Stop(bola sudah dekat)
		countReadyKick++;
	} else { //Kejar Bola(bola masih jauh)
		countReadyKick = 0;
	}

	if (countReadyKick >= 1) { //5
		PxMove = 0.0; //jalan ditempat
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.30; //0.045
	} else {
		if (headTilt < -1.5) {
			PxMove = kejarMax; //0.08
		} else if (headTilt >= -1.5 && headTilt < -1.4) {
			PxMove = kejarMid; //0.07
		} else if (headTilt > -1.0) {
			PxMove = lari; //0.05
		} else {
			PxMove = kejar; //0.06
		}
		//PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
		//PxMove = 0.06 / -1.6 * posTilt; //0.04-0.06
		PyMove = errorfPan * 0.50;//0.125; //0.045
		PaMove = errorfPan * 0.45;//0.25; //0.35; //0.045
	}

	if (mode == 0) { // Mode differential walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
	else if (mode == 1) { // Mode omnidirectional walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
			Walk(PxMove, PyMove, PaMove);
		} else { 					//printf("DDDDDDDD\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
}


// Arah Gawang dalam GameController ==========================================================
int	uniV = 0;
void arahGoal() {
	if (strategy.Uniform == 0) { // ketika display gamacontroller searah dengan juri
		if (gameController.timNumber1 == 0 && gameController.timColour1 == 0 && gameController.timNumber2 == 0 && gameController.timColour2 == 0) {
			uniV = 0;
		} else {
			if (gameController.FirstHalf == 1) { // Babak pertama
				if (gameController.timNumber1 == TEAM) { uniV = 0; }
				else if (gameController.timNumber2 == TEAM) { uniV = 1; }
				//if (gameController.timNumber1 == TEAM && gameController.timColour1 == 0) { uniV = 0; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 1) { uniV = 0; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 0) { uniV = 1; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 1) { uniV = 1; }
			} else { // Babak kedua
				if (gameController.timNumber1 == TEAM) { uniV = 0; }
				else if (gameController.timNumber2 == TEAM) { uniV = 1; }
				//if (gameController.timNumber2 == TEAM && gameController.timColour2 == 0) { uniV = 1; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 1) { uniV = 1; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 0) { uniV = 0; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 1) { uniV = 0; }
			}
		}
	} else { // ketika display gamecontroller berlawanan arah dengan juri
		if (gameController.timNumber1 == 0 && gameController.timColour1 == 0 && gameController.timNumber2 == 0 && gameController.timColour2 == 0) {
			uniV = 1;
		} else {
			if (gameController.FirstHalf == 1) { // Babak pertama
				if (gameController.timNumber1 == TEAM) { uniV = 1; }
				else if (gameController.timNumber2 == TEAM) { uniV = 0; }
				//if (gameController.timNumber1 == TEAM && gameController.timColour1 == 0) { uniV = 1; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 1) { uniV = 1; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 0) { uniV = 0; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 1) { uniV = 0; }
			} else { // Babak kedua
				if (gameController.timNumber1 == TEAM) { uniV = 1; }
				else if (gameController.timNumber2 == TEAM) { uniV = 0; }
				//if (gameController.timNumber2 == TEAM && gameController.timColour2 == 0) { uniV = 0; }
				//else if (gameController.timNumber2 == TEAM && gameController.timColour2 == 1) { uniV = 0; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 0) { uniV = 1; }
				//else if (gameController.timNumber1 == TEAM && gameController.timColour1 == 1) { uniV = 1; }
			}
		}
	}
}

// Set Arah Kompas ===============================================================================
int	Utara,//		= 200,//0,		//  goalAway /  arah depan
	Timur,//		= 309,//90,		//              arah kanan
	Selatan,//		= 30,//180,		//  goalHome /  arah belakang
	Barat,//		= 136,//270,		//              arah kiri
	lastNumber = 0;

void arah() {
	if (strategy.strategyNumber != lastNumber) {
		if (strategy.strategyNumber == 15) {
			if (useAutoDirection) {
				arahGoal();
				if (uniV == 0) {
					Utara = strategy.bearingValue;
					if ((Utara + 90) > 360) { Timur = 90 - (360 - Utara); }
						else { Timur = Utara + 90; }
					if ((Timur + 90) > 360) { Selatan = 90 - (360 - Timur); }
						else { Selatan = Timur + 90; }
					if ((Selatan + 90) > 360) { Barat = 90 - (360 - Selatan); }
						else { Barat = Selatan + 90; }
				} else if (uniV == 1) {
					Selatan = strategy.bearingValue;
					if ((Selatan + 90) > 360) { Barat = 90 - (360 - Selatan); }
						else { Barat = Selatan + 90; }
					if ((Barat + 90) > 360) { Utara = 90 - (360 - Barat); }
						else { Utara = Barat + 90; }
					if ((Utara + 90) > 360) { Timur = 90 - (360 - Utara); }
						else { Timur = Utara + 90; }
				}
			} else {
				if (strategy.Uniform == 0) {
					Utara = strategy.bearingValue;
					if ((Utara + 90) > 360) { Timur = 90 - (360 - Utara); }
						else { Timur = Utara + 90; }
					if ((Timur + 90) > 360) { Selatan = 90 - (360 - Timur); }
						else { Selatan = Timur + 90; }
					if ((Selatan + 90) > 360) { Barat = 90 - (360 - Selatan); }
						else { Barat = Selatan + 90; }
				} else if (strategy.Uniform == 1) {
					Selatan = strategy.bearingValue;
					if ((Selatan + 90) > 360) { Barat = 90 - (360 - Selatan); }
						else { Barat = Selatan + 90; }
					if ((Barat + 90) > 360) { Utara = 90 - (360 - Barat); }
						else { Utara = Barat + 90; }
					if ((Utara + 90) > 360) { Timur = 90 - (360 - Utara); }
						else { Timur = Utara + 90; }
				}
			}
		} lastNumber = strategy.strategyNumber;
	}
}

// Arah Gawang dalam bearingValue ============================================================
void goalDirec(int macau) {
	if (useAutoDirection) {
		arahGoal();
		if (uniV == 0) { myGoal = Utara + macau; }
		else { myGoal = Selatan + macau; }
	} else {
		if (strategy.Uniform == 0) { myGoal = Utara + macau; }
		else { myGoal = Selatan + macau; }
	}
}

// Variable kompas ===========================================================================
int	perpindahanMutlak; // adalah perpindahan nilai kompas 0-360
	// KET : Putaran kuadran adalah ClockWise
	// 1 adalah kuadran A (Utara   -  Timur)
	// 2 adalah kuadran B (Timur   -  Selatan)
	// 3 adalah kuadran C (Selatan -  Barat)
	// 4 adalah kuadran D (Barat   -  Utara)

	// pengali : Bearing 96 adalah sudut 0*, sedangkan Bearing 241 adalah sudut 90*, maka selisih satu nilai Bearing adalah berapa besar sudut???
double	pengaliA, // daerah kuadran a
	pengaliB, // daerah kuadran b
	pengaliC, // daerah kuadran c
	pengaliD, // daerah kuadran d

	selisihA,
	selisihB,
	selisihC,
	selisihD,
	nK, nT, suTam, suHir;

// Konversi Kompas Ke Sudut ==================================================================
void sudut() {
/*	if (Utara > Timur && Selatan > Timur && Barat > Selatan && Utara > Barat) {
		perpindahanMutlak = 1;
	} else if (Timur > Selatan && Barat > Selatan && Utara > Barat && Timur > Utara) {
		perpindahanMutlak = 2;
	} else if (Selatan > Barat && Utara > Barat && Timur > Utara && Selatan > Timur) {
		perpindahanMutlak = 3;
	} else if (Barat > Utara && Timur > Utara && Selatan > Timur && Barat > Selatan) {
		perpindahanMutlak = 4;
	} else {
		printf("  IMU SALAH INPUT..............................................................!!!!!!!!!!!!!!!!!!!!!\n");
		system("killall screen;");
	}


	if (perpindahanMutlak == 1) { selisihA = abs((360-Utara)+Timur); }
		else { selisihA = abs(Utara-Timur); }
	if (perpindahanMutlak == 2) { selisihB = abs((360-Timur)+Selatan); }
		else { selisihB = abs(Timur-Selatan); }
	if (perpindahanMutlak == 3) { selisihC = abs((360-Selatan)+Barat); }
		else { selisihC = abs(Selatan-Barat); }
	if (perpindahanMutlak == 4) { selisihD = abs((360-Barat)+Utara); }
		else { selisihD = abs(Barat-Utara); }

	pengaliA = 90/selisihA;
	pengaliB = 90/selisihB;
	pengaliC = 90/selisihC;
	pengaliD = 90/selisihD;

	if (perpindahanMutlak == 1) {
		//Kuadran A
		if ( strategy.bearingValue >= Utara || strategy.bearingValue <= Timur ) { // antara 0 - 90
			if (perpindahanMutlak == 1) {
				if (strategy.bearingValue >= Utara && strategy.bearingValue <= 360) {
					angle = ((strategy.bearingValue - Utara) * pengaliA);
				} else {
					angle = ((selisihA-(Timur - strategy.bearingValue)) * pengaliA);
				}
			} else {
				angle = ((strategy.bearingValue - Utara) * pengaliA);
			}
		}

		//Kuadran B
		else if ( strategy.bearingValue >= Timur && strategy.bearingValue <= Selatan ) { // antara 90 - 180
			if (perpindahanMutlak == 2) {
				if (strategy.bearingValue >= Timur && strategy.bearingValue <= 360) {
					angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
				} else {
					angle = (((selisihB-(Selatan - strategy.bearingValue)) * pengaliB) + 90);
				}
			} else {
				angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
			}
		}

		//Kuadran C
		else if ( strategy.bearingValue >= Selatan && strategy.bearingValue <= Barat ) { // antara (-90) - (-180) ============= Change
			if (perpindahanMutlak == 3) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
				} else {
					angle = -(((selisihC-(strategy.bearingValue - Selatan)) * pengaliC) + 90);
				}
			} else {
				angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
			}
		}

		//Kuadran D
		else if ( strategy.bearingValue >= Barat && strategy.bearingValue <= Utara ) { // antara 0 - (-90)
			if (perpindahanMutlak == 4) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -((Utara - strategy.bearingValue) * pengaliD);
				} else {
					angle = -((selisihD-(strategy.bearingValue - Barat)) * pengaliD);
				}
			} else {
				angle = -((Utara - strategy.bearingValue) * pengaliD);
			}
		}
	} else if (perpindahanMutlak == 2) {
		//Kuadran A
		if ( strategy.bearingValue >= Utara && strategy.bearingValue <= Timur ) { // antara 0 - 90
			if (perpindahanMutlak == 1) {
				if (strategy.bearingValue >= Utara && strategy.bearingValue <= 360) {
					angle = ((strategy.bearingValue - Utara) * pengaliA);
				} else {
					angle = ((selisihA-(Timur - strategy.bearingValue)) * pengaliA);
				}
			} else {
				angle = ((strategy.bearingValue - Utara) * pengaliA);
			}
		}

		//Kuadran B
		else if ( strategy.bearingValue >= Timur || strategy.bearingValue <= Selatan ) { // antara 90 - 180
			if (perpindahanMutlak == 2) {
				if (strategy.bearingValue >= Timur && strategy.bearingValue <= 360) {
					angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
				} else {
					angle = (((selisihB-(Selatan - strategy.bearingValue)) * pengaliB) + 90);
				}
			} else {
				angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
			}
		}

		//Kuadran C
		else if ( strategy.bearingValue >= Selatan && strategy.bearingValue <= Barat ) { // antara (-90) - (-180) ============= Change
			if (perpindahanMutlak == 3) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
				} else {
					angle = -(((selisihC-(strategy.bearingValue - Selatan)) * pengaliC) + 90);
				}
			} else {
				angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
			}
		}

		//Kuadran D
		else if ( strategy.bearingValue >= Barat && strategy.bearingValue <= Utara ) { // antara 0 - (-90)
			if (perpindahanMutlak == 4) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -((Utara - strategy.bearingValue) * pengaliD);
				} else {
					angle = -((selisihD-(strategy.bearingValue - Barat)) * pengaliD);
				}
			} else {
				angle = -((Utara - strategy.bearingValue) * pengaliD);
			}
		}
	} else if (perpindahanMutlak == 3) {
		//Kuadran A
		if ( strategy.bearingValue >= Utara && strategy.bearingValue <= Timur ) { // antara 0 - 90
			if (perpindahanMutlak == 1) {
				if (strategy.bearingValue >= Utara && strategy.bearingValue <= 360) {
					angle = ((strategy.bearingValue - Utara) * pengaliA);
				} else {
					angle = ((selisihA-(Timur - strategy.bearingValue)) * pengaliA);
				}
			} else {
				angle = ((strategy.bearingValue - Utara) * pengaliA);
			}
		}

		//Kuadran B
		else if ( strategy.bearingValue >= Timur && strategy.bearingValue <= Selatan ) { // antara 90 - 180
			if (perpindahanMutlak == 2) {
				if (strategy.bearingValue >= Timur && strategy.bearingValue <= 360) {
					angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
				} else {
					angle = (((selisihB-(Selatan - strategy.bearingValue)) * pengaliB) + 90);
				}
			} else {
				angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
			}
		}

		//Kuadran C
		else if ( strategy.bearingValue >= Selatan || strategy.bearingValue <= Barat ) { // antara (-90) - (-180) ============= Change
			if (perpindahanMutlak == 3) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
				} else {
					angle = -(((selisihC-(strategy.bearingValue - Selatan)) * pengaliC) + 90);
				}
			} else {
				angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
			}
		}

		//Kuadran D
		else if ( strategy.bearingValue >= Barat && strategy.bearingValue <= Utara ) { // antara 0 - (-90)
			if (perpindahanMutlak == 4) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -((Utara - strategy.bearingValue) * pengaliD);
				} else {
					angle = -((selisihD-(strategy.bearingValue - Barat)) * pengaliD);
				}
			} else {
				angle = -((Utara - strategy.bearingValue) * pengaliD);
			}
		}
	} else if (perpindahanMutlak == 4) {
		//Kuadran A
		if ( strategy.bearingValue >= Utara && strategy.bearingValue <= Timur ) { // antara 0 - 90
			if (perpindahanMutlak == 1) {
				if (strategy.bearingValue >= Utara && strategy.bearingValue <= 360) {
					angle = ((strategy.bearingValue - Utara) * pengaliA);
				} else {
					angle = ((selisihA-(Timur - strategy.bearingValue)) * pengaliA);
				}
			} else {
				angle = ((strategy.bearingValue - Utara) * pengaliA);
			}
		}

		//Kuadran B
		else if ( strategy.bearingValue >= Timur && strategy.bearingValue <= Selatan ) { // antara 90 - 180
			if (perpindahanMutlak == 2) {
				if (strategy.bearingValue >= Timur && strategy.bearingValue <= 360) {
					angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
				} else {
					angle = (((selisihB-(Selatan - strategy.bearingValue)) * pengaliB) + 90);
				}
			} else {
				angle = (((strategy.bearingValue - Timur) * pengaliB) + 90);
			}
		}

		//Kuadran C
		else if ( strategy.bearingValue >= Selatan && strategy.bearingValue <= Barat ) { // antara (-90) - (-180) ============= Change
			if (perpindahanMutlak == 3) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
				} else {
					angle = -(((selisihC-(strategy.bearingValue - Selatan)) * pengaliC) + 90);
				}
			} else {
				angle = -(((Barat - strategy.bearingValue) * pengaliC) + 90);
			}
		}

		//Kuadran D
		else if ( strategy.bearingValue >= Barat || strategy.bearingValue <= Utara ) { // antara 0 - (-90)
			if (perpindahanMutlak == 4) {
				if (strategy.bearingValue <= Barat && strategy.bearingValue >= 0) {
					angle = -((Utara - strategy.bearingValue) * pengaliD);
				} else {
					angle = -((selisihD-(strategy.bearingValue - Barat)) * pengaliD);
				}
			} else {
				angle = -((Utara - strategy.bearingValue) * pengaliD);
			}
		}
	}
	if (useAutoDirection) {
		arahGoal();
		if (uniV == 1) {
			if (angle > 0) { angle = angle - 180; }
			else { angle = angle + 180; }
		}
	} else {
		if (strategy.Uniform == 1) {
			if (angle > 0) { angle = angle - 180; }
			else { angle = angle + 180; }
		}
	}
*/
	angle = strategy.bearingValue;
/*	nK = 6;
	nT = 6/90;
	suTam = angle * nT;
	suHir = angle + suTam;
*/
	odometry.RobotAngle = angle;
	if (angle > 0){
		sendAngle = angle;
	} else if (angle < 0){
		sendAngle = angle + 360;
	} else if (angle == 0){
		sendAngle =  0;
	}
	//printf("  Sudut = %.f", angle);
}

// Sudut2 ============================================================================
double	angle2 = 0,
	bearingValue2 = 0,
	setPointAngles = 0;
void sudut2() {
	getSensor(); //angleZ -> kekanan = negatif, kekiri = positif
	bearingValue2 = setPointAngles - angleZ;
	if (bearingValue2 > 360) {
		setPointAngles = angleZ;
		bearingValue2 = 0;
	} else if (bearingValue2 < 0) {
		setPointAngles = angleZ;
		bearingValue2 = 360;
	} //printf("bearingValue2 = %.f", bearingValue2);

	if (bearingValue2 >= 0 && bearingValue2 < 180) {
		angle2 = bearingValue2;
	} else {
		angle2 = bearingValue2 - 360;
	} //printf("angle2 = %.f", angle2);
}

// IMU ===========================================================================
int	errorBodyPos,
	delayImu = 0,
	delayReset = 0,
	bearingValue,
	setPoint1,
	setPoint2,
	setPointAngle = 0,
	setAngle = 0;

double	errorCPosPan,
	errorCPosTilt,
	errorCPosAlfa,
	alfaImu,
	bodyYImu,
	bodyXImu,
	pengaliAlfa;

//	X+ = maju
//	X- = mundur
//	Y+ = samping kiri
//	Y- = samping kanan
//	A+ = putar kiri
//	A- = putar kanan

void rotateDirec(int arah, double jarakTilt) {
	errorCPosPan = posPan;// adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
	errorCPosTilt = posTilt - (jarakTilt);//-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi

	//bodyXImu 	= errorCPosTilt * (-0.1);
	//bodyYImu 	= 0.017;			//max = 0.03, min = 0.017
	//alfaImu 	= -0.14;			//max = -0.2, min = -0.14

	bodyXImu 	= errorCPosTilt * (-0.1);	//nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
	bodyYImu 	= abs(errorCPosPan/100) + 0.018;//0.017;
	//bodyYImu 	= 0.02 + (errorCPosPan*0.025);	//0.017;
	alfaImu 	= errorCPosPan * 0.7;		//0.7; nilai pengali ini harus tetap bernilai positif //besarnya kalkulasi rotate yg dibutuhkan tehadap posPan

	if (arah <= 0) { //rotate ke kanan
		//alfaImu = -0.15; bodyYImu = 0.017;
		//if (alfaImu > 0) { alfaImu = -alfaImu; }
		if (bodyYImu < 0) { bodyYImu = -bodyYImu; }
	} else { //rotate ke kiri
		//alfaImu = 0.15; bodyYImu = -0.017;
		//if (alfaImu < 0) { alfaImu = -alfaImu; }
		if (bodyYImu > 0) { bodyYImu = -bodyYImu; }
	}

	//printf("  bodyXImu = %.4f, bodyYImu = %.4f, alfaImu = %.4f, sudut = %.f,\n", bodyXImu, bodyYImu, alfaImu, angle);
}

//main
void rotateDirec2(int setSudut, double jarakTilt) {
	errorCPosPan = posPan;// adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
	errorCPosTilt = jarakTilt - posTilt;//-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi
	//errorCPosAlfa = angle; //-180 0 180
	if (setSudut >= 0) { //setSudut lebih besar dari nol
		if (angle <= setSudut && angle >= (setSudut-180)) { //range 180 kiri (negatif)
			//if (angle < 0) { //angle berada di posisi minus
				errorCPosAlfa = angle - setSudut;
			//} else { //angle berada di posisi plus
			//	errorCPosAlfa = angle - setSudut;
			//}
		} else { //range 180 kanan (positif)
			if (angle < 0) { //angle berada di posisi minus
				errorCPosAlfa = (180-setSudut) + (angle+180);
			} else { //angle berada di posisi plus
				errorCPosAlfa = angle - setSudut;
			}
		}
	} else { //setSudut lebih kecil dari nol
		if (angle >= setSudut && angle <= (setSudut+180)) { //range 180 kanan (positif)
			//if (angle < 0) { //angle berada di posisi minus
				errorCPosAlfa = angle - setSudut;
			//} else { //angle berada di posisi plus
			//	errorCPosAlfa = angle - setSudut;
			//}
		} else { //range 180 kiri (negatif)
			if (angle < 0) { //angle berada di posisi minus
				errorCPosAlfa = angle - setSudut;
			} else { //angle berada di posisi plus
				errorCPosAlfa = (angle-180) - (180+setSudut);
			}
		}
	}

	// X ============================================================================================
	if (errorCPosTilt > 0) {
		bodyXImu = errorCPosTilt * (0.1);				
	} else {
		//if (robotID == 5) { bodyXImu = errorCPosTilt * (0.1);	}			
		//else {
			bodyXImu = errorCPosTilt * (0.2);
		//}				
	}
	if (bodyXImu > 0.06) { bodyXImu = 0.06; } //batas maksimal x
	else if (bodyXImu < -0.03) { bodyXImu = -0.03; } //batas minimal x

	// A ============================================================================================
	//"errorCPosAlfa, pengurang: errorCPosTilt dan errorCPosPan"
	//pengaliAlfa	= 0.002;
	//pengaliAlfa	= 0.004444444;

	//pengaliAlfa	= 0.004 - (errorCPosTilt * 0.0075) + (errorCPosPan * 0.003);  //Y=0.2(0.0015) X=0.4(0.0015)
	pengaliAlfa	= 0.004 - (errorCPosTilt * 0.005) + (errorCPosPan * 0.003);   //Y=0.3(0.0015) X=0.4(0.0015)
	//pengaliAlfa	= 0.004 - (errorCPosTilt * 0.00375) + (errorCPosPan * 0.003); //Y=0.4(0.0015) X=0.4(0.0015)

	//if (errorCPosTilt > 0) {
	//	pengaliAlfa	= 0.004 - (errorCPosTilt * 0.0075) + (errorCPosPan * 0.0025);  //Y=0.2(0.0015) X=0.4(0.0015)
	//} else {
	//	pengaliAlfa	= 0;
	//	pengaliAlfa	= 0.004 - (errorCPosPan * 0.0025);  //X=0.4(0.0015)
	//}

	if (pengaliAlfa < 0) { pengaliAlfa = 0; }
	alfaImu 	= errorCPosAlfa * pengaliAlfa;				
	if (errorCPosTilt < 0.15) {
		if (alfaImu > -0.14 && alfaImu < 0.14) { //batas minimal a
			if (alfaImu > 0) { alfaImu = 0.14; }
			else if (alfaImu < 0) { alfaImu = -0.14; }
		}
	}
	if (alfaImu < -0.2 || alfaImu > 0.2) { //batas maksimal a
		if (alfaImu > 0) { alfaImu = 0.2; }
		else { alfaImu = -0.2; }
	}

	// Y ============================================================================================
	//bodyYImu 	= alfaImu * 0.121428571;
	//bodyYImu 	= alfaImu * 0.15;

	//bodyYImu 	= (alfaImu * -0.15) + (errorCPosPan * 0.025); //0.05 //0.025 //0.01
	bodyYImu 	= (errorCPosAlfa * -0.0006) + (errorCPosPan * 0.025);		
	//if (bodyYImu >= -0.017 && bodyYImu <= 0.017) { //batas maksimal Y
	//	if (bodyYImu > 0) { bodyYImu = 0.017; }
	//	else { bodyYImu = -0.017; }
	//} else if (bodyYImu < -0.03 || bodyYImu > 0.03) { //batas maksimal Y
	//	if (bodyYImu > 0) { bodyYImu = 0.03; }
	//	else { bodyYImu = -0.03; }
	//}

	//printf("  bodyXImu = %.4f, bodyYImu = %.4f, alfaImu = %.4f, sudut = %.f,\n", bodyXImu, bodyYImu, alfaImu, angle);
}

void rotateParabolic(int arah, double jarakTilt) {
	rotateDirec(arah, jarakTilt);
	Walk(bodyXImu, bodyYImu, alfaImu);
}

bool	robotDirection = false;
int	eleh = 0,
	btsRotate = 0,
	btsSetPoint = 0;
void Imu(int gawang, double jarakTilt) {// opsi 2, menggunakan "mode" untuk mengecek robot direction
	trackBall();

	if (gawang > 180) { gawang = 180; }
	else if (gawang < -180) { gawang = -180; }

	//goalDirec(gawang); //Dipakai jika menggunakan magnetometer
	sudut();

	setPoint1 =  10 + gawang;//10 //15 //20 //60
	setPoint2 = -10 + gawang;//10 //15 //20 //60

//mode1
/*	setAngle = gawang;
	if (angle >= setPoint2 && angle <= setPoint1) {
		bodyXImu = bodyYImu = alfaImu = 0.0;
		if (eleh > 10) { robotDirection = true;	}
		else { eleh++; }
	} else {
		//KUADRAN I ============================================================================
		if (gawang >= 0 && gawang < 90) {
			setPointAngle = gawang - 180;

			//kuadaran I
			if (angle >= 0 && angle < 90) {
				if (angle < gawang) { //printf(" rotate ke kanan\n\n");
					rotateDirec(-1, jarakTilt);
				} else { //printf(" rotate ke kiri\n\n");
					rotateDirec(1, jarakTilt);
				}
			}
			//kuadaran II
			else if (angle >= -90 && angle < 0) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			//kuadaran III
			else if (angle >= setPointAngle && angle < -90) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			else if (angle >= -180 && angle < setPointAngle) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			//kuadaran IV
			else if (angle >= 90 && angle < 180) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}

		//KUADRAN II ============================================================================
		} else if (gawang >= -90 && gawang < 0) {
			setPointAngle = gawang + 180;

			//kuadaran I
			if (angle >= 0 && angle < 90) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			//kuadaran II
			else if (angle >= -90 && angle < 0) {
				if (angle < gawang) { //printf(" rotate ke kanan\n\n");
					rotateDirec(-1, jarakTilt);
				} else { //printf(" rotate ke kiri\n\n");
					rotateDirec(1, jarakTilt);
				}
			}
			//kuadaran III
			else if (angle >= -180 && angle < -90) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			//kuadaran IV
			else if (angle >= 90 && angle < setPointAngle) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			else if (angle >= setPointAngle && angle < 180) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}

		//KUADRAN III ============================================================================
		} else if (gawang >= -180 && gawang < 90) {
			setPointAngle = gawang + 180;

			//kuadaran I
			if (angle >= 0 && angle < setPointAngle) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			else if (angle >= setPointAngle && angle < 90) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			//kuadaran II
			else if (angle >= -90 && angle < 0) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			//kuadaran III
			else if (angle >= -180 && angle < -90) {
				if (angle < gawang) { //printf(" rotate ke kanan\n\n");
					rotateDirec(-1, jarakTilt);
				} else { //printf(" rotate ke kiri\n\n");
					rotateDirec(1, jarakTilt);
				}
			}
			//kuadaran IV
			else if (angle >= 90 && angle < 180) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}

		//KUADRAN IV ============================================================================
		} else if (gawang >= 90 && gawang < 180) {
			setPointAngle = gawang - 180;

			//kuadaran I
			if (angle >= 0 && angle < 90) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			//kuadaran II
			else if (angle >= setPointAngle && angle < 0) {
				rotateDirec(-1, jarakTilt); //printf(" rotate ke kanan\n\n");
			}
			else if (angle >= -90 && angle < setPointAngle) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			//kuadaran III
			else if (angle >= -180 && angle < -90) {
				rotateDirec(1, jarakTilt); //printf(" rotate ke kiri\n\n");
			}
			//kuadaran IV
			else if (angle >= 90 && angle < 180) {
				if (angle < gawang) { //printf(" rotate ke kanan\n\n");
					rotateDirec(-1, jarakTilt);
				} else { //printf(" rotate ke kiri\n\n");
					rotateDirec(1, jarakTilt);
				}
			}
		}

		eleh = 0;
		robotDirection = false;
	} Walk(bodyXImu, bodyYImu, alfaImu);*/

//mode2
	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah compas dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) {
				bodyXImu = bodyYImu = alfaImu = 0.0;
				if (eleh > 10) { robotDirection = true;	}
				else { eleh++; }
			} else  {
				if (gawang >= 0) {
					btsRotate = gawang - 180;
					if ((angle <= gawang) && (angle >= btsRotate)) {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					} else {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					}
				} else {
					btsRotate = gawang + 180;
					if ((angle >= gawang) && (angle <= btsRotate)) {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					} else {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					}
				}

				eleh = 0;
				robotDirection = false;
			} Walk(bodyXImu, bodyYImu, alfaImu);
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				bodyXImu = bodyYImu = alfaImu = 0.0;
				if (eleh > 10) { robotDirection = true;	}
				else { eleh++; }
			} else {
				if (gawang >= 0) {
					btsRotate = gawang - 180;
					if ((angle <= gawang) && (angle >= btsRotate)) {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					} else {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					}
				} else {
					btsRotate = gawang + 180;
					if ((angle >= gawang) && (angle <= btsRotate)) {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					} else {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					}
				}

				eleh = 0;
				robotDirection = false;
			} Walk(bodyXImu, bodyYImu, alfaImu);
		}
	} else { // arah kompas kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			bodyXImu = bodyYImu = alfaImu = 0.0;
			if (eleh > 10) { robotDirection = true;	}
			else { eleh++; }
		} else {
			if (gawang >= 0) {
				btsRotate = gawang - 180;
				if ((angle <= gawang) && (angle >= btsRotate)) {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				} else {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				}
			} else {
				btsRotate = gawang + 180;
				if ((angle >= gawang) && (angle <= btsRotate)) {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				} else {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				}
			}

			eleh = 0;
			robotDirection = false;
		} Walk(bodyXImu, bodyYImu, alfaImu);
	}

//mode3
/*	if (headTilt < -1.0) {											
		if (angle >= setPoint2 && angle <= setPoint1) {
			bodyXImu = bodyYImu = alfaImu = 0.0;
			if (eleh > 10) { robotDirection = true;	}
			else { eleh++; }
		} else {
			if (gawang >= 0) {
				btsRotate = gawang - 180;
				if ((angle <= gawang) && (angle >= btsRotate)) {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				} else {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				}
			} else {
				btsRotate = gawang + 180;
				if ((angle >= gawang) && (angle <= btsRotate)) {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				} else {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				}
			}

			eleh = 0;
			robotDirection = false;
		} Walk(bodyXImu, bodyYImu, alfaImu);
	} else {
		if (angle >= setPoint2 && angle <= setPoint1) {
			bodyXImu = bodyYImu = alfaImu = 0.0;
			if (eleh > 10) { robotDirection = true; }
			else { eleh++; }
		} else {
			eleh = 0;
			robotDirection = false;
			rotateDirec2(gawang, jarakTilt);
		} Walk(bodyXImu, bodyYImu, alfaImu);
	}*/
}

// Grid Coordinate ================================================================
double outGrid;
void gridCoor(){
	if (robotPos_X <= -225){
		outGrid = 0;
	} else if (robotPos_X > -225 && robotPos_X < 225){
		outGrid = odometry.ArchSin;
	} else if (robotPos_X >= 225){
		if (robotPos_Y <=-210){
			outGrid = 60;
		} else if (robotPos_Y > -210 && robotPos_Y <=-120){
			outGrid = 40;
		} else if (robotPos_Y > -120 && robotPos_Y <=-40){
			outGrid = 20;
		} else if (robotPos_Y > -40 && robotPos_Y <=40){
			outGrid = 0;
		} else if (robotPos_Y > 40 && robotPos_Y <=120){
			outGrid = -20;
		} else if (robotPos_Y > 120 && robotPos_Y <=210){
			outGrid = -40;
		} else if (robotPos_Y > 210){
			outGrid = -60;
		}
	}

	//untuk kondisi followSearchGoal
	if (Activated) {
		if (robotPos_X < 225) followSearchAktif = true;
		else followSearchAktif = false;
	}
}

// Rotate Body with IMU ===========================================================
bool	posRotate = false;

void rotateBodyImu(int rotate) {
	trackBall();

	//goalDirec(rotate); //Dipakai jika menggunakan magnetometer
	sudut();

	setPoint1 =  5 + rotate; //20 bisa kemungkinan 180 keatas
	setPoint2 = -5 + rotate; //20 bisa kemungkinan -180 bawah

	if (angle > setPoint2 && angle < setPoint1 ) {
		posRotate = true;
	} else {
		if (angle > rotate) { //putar Kiri
			Walk(0.0, 0.0, 0.2);
		}  else { //putar Kanan
			Walk(0.0, 0.0, -0.2);
		}
		//posRotate = false;
	}
}

// Rotate Body with IMU ===========================================================
bool	posRotateNew = false;
double	IaMove, errorImuNew,
	putar;
void rotateBodyImuNew(int rotate) {
	trackBall();

	//goalDirec(rotate); //Dipakai jika menggunakan magnetometer
	sudut();

	/*setPoint1 =  5 + rotate; //20
	setPoint2 = -5 + rotate; //20
	if(setPoint1>180){ setPoint1 = 180;}
	else if(setPoint1<-180){setPoint1 = -180;}

	if(setPoint2>180){ setPoint2 = 180;}
	else if(setPoint2<-180){setPoint2 = -180;}

	errorImuNew = angle - rotate;
	IaMove = errorImuNew * 0.0065;

	if (angle > setPoint2 && angle < setPoint1 ) {
		posRotateNew = true;
	} else {
		if (angle > rotate) { //putar Kiri
			Walk(0.0, 0.0, IaMove);
		}  else { //putar Kanan
			Walk(0.0, 0.0, -IaMove);
		}
		//posRotate = false;
	}*/
	setPoint1 =  5 + rotate;//10 //15 //20 //60
	setPoint2 = -5 + rotate;//10 //15 //20 //60

	//putar = 0.2;
	//errorFollow = angle - rotate;
	//putar = errorFollow * 0.004; //0.0033

	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah compas dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) { //misal 170 ke -170
				PaMove = 0.00;
				posRotateNew = true;
			} else  {
				//if (rotate >= 0) { //misal 180
					btsRotate = rotate - 180;
					if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
						putar = (rotate - angle) * 0.0065; //0.0033
						PaMove = -putar;
					} else { //putar kiri
						if (angle > rotate) { putar = (angle - rotate) * 0.0065; } //0.0033
						else { putar = ((180 - rotate) + (180 + angle)) * 0.0065; } //0.0033
						//putar = 0.2;
						PaMove = putar;
					}
				//} else { //misal -180
				//	btsRotate = rotate + 180;
				//	if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
				//		putar = abs(rotate - angle) * 0.004; //0.0033
				//		PaMove = putar;
				//	} else { //putar kanan
				//		if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
				//		else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
				//		//putar = 0.2;
				//		PaMove = -putar;
				//	}
				//}
			}
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				PaMove = 0.00;
				posRotateNew = true;
			} else {
				//if (rotate >= 0) {
				//	btsRotate = rotate - 180;
				//	if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
				//		putar = (rotate - angle) * 0.004; //0.0033
				//		PaMove = -putar;
				//	} else { //putar kiri
				//		if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
				//		else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
				//		//putar = 0.2;
				//		PaMove = putar;
				//	}
				//} else { //misal -180
					btsRotate = rotate + 180;
					if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
						putar = abs(rotate - angle) * 0.0065; //0.0033
						PaMove = putar;
					} else { //putar kanan
						if (angle < rotate) { putar = (rotate - angle) * 0.0065; } //0.0033
						else { putar = ((180 + rotate) + (180 - angle)) * 0.0065; } //0.0033
						//putar = 0.2;
						PaMove = -putar;
					}
				//}
			}
		}
	} else { // arah kompas kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			PaMove = 0.00;
			posRotateNew = true;
		} else {
			if (rotate >= 0) {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //putar kanan
					putar = (rotate - angle) * 0.0065; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.0065; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.0065; } //0.0033
					//putar = 0.2;
					PaMove = putar;
				}
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) {//maka putar kiri
					putar = abs(rotate - angle) * 0.0065; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.0065; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.0065; } //0.0033
					//putar = 0.2;
					PaMove = -putar;
				}
			}
		}
	}
	if(!posRotateNew){
		Walk(0.0,0.0,PaMove);
	}

}


//	A+ = putar kiri
//	A- = putar kanan
void jalanDirection(double Xwalk, double Ywalk, double rotate) {
	if (rotate > 180) { rotate = 180; }
	else if (rotate < -180) { rotate = -180; }

	setPoint1 =  5 + rotate;//10 //15 //20 //60
	setPoint2 = -5 + rotate;//10 //15 //20 //60

	//putar = 0.2;
	//errorFollow = angle - rotate;
	//putar = errorFollow * 0.004; //0.0033

	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah compas dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) { //misal 170 ke -170
				PaMove = 0.00;
			} else  {
				//if (rotate >= 0) { //misal 180
					btsRotate = rotate - 180;
					if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
						putar = (rotate - angle) * 0.0065; //0.0033
						PaMove = -putar;
					} else { //putar kiri
						if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
						else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
						//putar = 0.2;
						PaMove = putar;
					}
				//} else { //misal -180
				//	btsRotate = rotate + 180;
				//	if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
				//		putar = abs(rotate - angle) * 0.004; //0.0033
				//		PaMove = putar;
				//	} else { //putar kanan
				//		if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
				//		else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
				//		//putar = 0.2;
				//		PaMove = -putar;
				//	}
				//}
			}
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				PaMove = 0.00;
			} else {
				//if (rotate >= 0) {
				//	btsRotate = rotate - 180;
				//	if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
				//		putar = (rotate - angle) * 0.004; //0.0033
				//		PaMove = -putar;
				//	} else { //putar kiri
				//		if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
				//		else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
				//		//putar = 0.2;
				//		PaMove = putar;
				//	}
				//} else { //misal -180
					btsRotate = rotate + 180;
					if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
						putar = abs(rotate - angle) * 0.004; //0.0033
						PaMove = putar;
					} else { //putar kanan
						if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
						else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
						//putar = 0.2;
						PaMove = -putar;
					}
				//}
			}
		}
	} else { // arah kompas kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			PaMove = 0.00;
		} else {
			if (rotate >= 0) {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //putar kanan
					putar = (rotate - angle) * 0.004; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
					//putar = 0.2;
					PaMove = putar;
				}
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) {//maka putar kiri
					putar = abs(rotate - angle) * 0.004; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
					//putar = 0.2;
					PaMove = -putar;
				}
			}
		}
	}

	if (PaMove > 0.3) { PaMove = 0.3; }
	else if (PaMove < -0.3) { PaMove = -0.3; }

	Walk(Xwalk, Ywalk, PaMove);
}

// Ball Positioning Using P Controller =======================================================
double	errorPosX,
	errorPosY,
	PxMoveBallPos,
	PyMoveBallPos,
	PaMoveBallPos;
bool	ballPos = false;
void ballPositioning(double setPointX, double setPointY, double speed) {
	errorPosX = headPan - setPointX;
	errorPosY = headTilt - setPointY;

	if ((errorPosX > -0.10 && errorPosX < 0.10) && (errorPosY > -0.10)) { //&& errorPosY < 0.10)) { //sudah sesuai
		PyMoveBallPos = 0.00;
		PxMoveBallPos = 0.00;
		ballPos = true;
	} else { //belum sesuai
		ballPos = false;
		if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) { //bola disamping //pan tilt kircok (polar)
			PxMoveBallPos = -0.03;
			PyMoveBallPos = errorPosX * 0.08;//0.12;
		} else {
//			if ((headPan >= 0.04 && headTilt > setPointY) || (headPan <= -0.04 && headTilt > setPointY)) { //bola disamping //pan tilt kircok (polar)
//				PxMoveBallPos = -0.03;
//			} else {
				//Xmove
				if (headTilt > setPointY) { //> (setPointY + 0.1)) { //kelebihan
					PxMoveBallPos = -0.03;
				} else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
					PxMoveBallPos = 0.00;
				} else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1)) { //bola sudah dekat
					PxMoveBallPos = errorPosY * -speed;
					//PxMoveBallPos = errorPosY * -0.08;
//					PxMoveBallPos = 0.01;
					if (PxMoveBallPos >= 0.025) { PxMoveBallPos = 0.025; }
					else if (PxMoveBallPos <= 0.00) { PxMoveBallPos = 0.00; }
				} else { //bola masih jauh
					PxMoveBallPos = headTilt * (0.08 / -1.6); //0.05
					//PxMoveBallPos = kejar;
//					PxMoveBallPos = 0.05;
				}

				//Ymove
				if (headTilt >= (setPointY - 0.03)) { //> (setPointY + 0.1)) { //kelebihan
					PyMoveBallPos = 0.00;
				} else {
					if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) { //sudah dalam range
						PyMoveBallPos = 0.00;
					} else { //belum dalam range
						PyMoveBallPos = errorPosX * 0.08;//0.08;//0.12;
					}
				}
//			}
		}

	} Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
}

// Dribble Ball ======================================================================
int	bawaBola;
double	setPointFootY, setPointFootY1, setPointFootY2;
void dribble(int gawang, double speed) {
	trackBall();

	//goalDirec(gawang); //Dipakai jika menggunakan magnetometer
	sudut();//

	//setPoint1 =  20 + gawang;//20
	//setPoint2 = -20 + gawang;//20

	//if (angle >= setPoint2 && angle <= setPoint1) {
	//	robotDirection = true;
	//} else  { robotDirection = false; }

	//if (robotDirection) { //printf("arah kompas sudah benar\n");
		//if (headTilt <= -0.7 || bawaBola >= 300) {
		//	bawaBola = 0;
		//	robotDirection = false;
		//	stateCondition = 5;
		//} bawaBola++;

		// Ball Positioning ======================================
		//if (posPan >= 0) { setPointFootY = 0.25;  }//kiri
		//else             { setPointFootY = -0.25; }//kanan
		//errorPosX = headPan - setPointFootY;

		setPointFootY1 =  0.2; //0.22
		setPointFootY2 = -0.2; //-0.22
		errorPosY = headTilt + 0.6;//0.04;//0.05;//0.08;

		//if(errorPosX >= -0.1 && errorPosX <= 0.1) {
		if(headPan <= setPointFootY1 && headPan >= setPointFootY2) { // -0.2 > x < 0.2
			//if (headTilt >= -0.8) {
				PxMoveBallPos = 0.3*speed;
			//} else {
			//	PxMoveBallPos = errorPosY*speed*-1;
			//}
			Walk(PxMoveBallPos, 0.0, 0.0);
		} else { // x < -0.2 || x > 0.2
			if (headTilt >= pTiltTendang) {
				PxMoveBallPos = errorPosY * -speed;
				//PxMoveBallPos = -0.02;//-0.03;
				//Walk(-0.03, 0.0, 0.0);
			} else {
				PxMoveBallPos = 0.0;
			}

			if (headPan > setPointFootY1) { //printf("kiri bos\n");
				PyMoveBallPos = (headPan - 0.1) * 0.06;
			} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
				PyMoveBallPos = (headPan + 0.1) * 0.06;
			}

			Walk(PxMoveBallPos, PyMoveBallPos, 0.0);

			//if (headTilt >= (cSekarang - 0.2)) {
			//	Walk(-0.03, 0.0, 0.0);
			//} else {
			//	if (headPan > setPointFootY1) { //printf("kiri bos\n");
			//		PyMoveBallPos = (headPan - 0.1) * 0.06;
			//		Walk(0.0, PyMoveBallPos, 0.0);
			//	} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
			//		PyMoveBallPos = (headPan + 0.1) * 0.06;
			//		Walk(0.0, PyMoveBallPos, 0.0);
			//	}
			//}

			//if(errorPosY >= -0.1) { // XmoveBackWard
			//	Walk(-0.03, 0.0, 0.0);
			//} else { // Ymove
			//	PyMoveBallPos = errorPosX*0.06;
			//	Walk(0.0, PyMoveBallPos, 0.0);
			//}
		}
	//} else { //printf("cari arah kompas\n");
	//	bawaBola = 0;
	//	if (posTilt > -0.8 && posPan > -0.5 && posPan < 0.5) {//bola dekat
	//		Imu(gawang, cSekarang);
	//	} else {//bola masih jauh
	//		followBall(0);
	//	}
	//}
}

void dribble2(int gawang, double speed) {
	trackBall();

	//goalDirec(gawang); //Dipakai jika menggunakan magnetometer
	sudut();//

		setPointFootY1 =  0.4; //0.22
		setPointFootY2 = -0.4; //-0.22
		errorPosY = headTilt + 0.6;//0.04;//0.05;//0.08;
		errorPosX = headPan - 0;

		if(headPan <= setPointFootY1 && headPan >= setPointFootY2) { // -0.4 > x < 0.4
			PxMoveBallPos = 0.3*speed;
			PyMoveBallPos = errorPosX * 0.045; //0.045
			PaMoveBallPos = errorPosX * 0.25; //0.35; //0.045

			Walk(PxMoveBallPos, 0.0, PaMoveBallPos);
		} else { // x < -0.4 || x > 0.4
			if (headTilt >= pTiltTendang) {
				PxMoveBallPos = errorPosY * -speed;
			} else {
				PxMoveBallPos = 0.0;
			}

			if (headPan > setPointFootY1) { //printf("kiri bos\n");
				PyMoveBallPos = (headPan - 0.1) * 0.06;
			} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
				PyMoveBallPos = (headPan + 0.1) * 0.06;
			}

			Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
		}
}

// Checking Lost Goal ========================================================================
int	countGoalLost  = 0,
	countGoalFound = 0,
	returnGoalVal;
int goalLost(int threshold) {
	if (useVision) {
		if (receler.Goal_X == -1 && receler.Goal_Y == -1) {
			countGoalFound = 0;
			countGoalLost++;
			if (countGoalLost >= threshold) {
				returnGoalVal = 1;
			}
		} else {
			if (headTilt < -1.0) {
				countGoalLost = 0;
				countGoalFound++;
				if (countGoalFound > 1) {
					returnGoalVal = 0;
				}
			}
		}
	} else {
		countGoalFound = 0;
		countGoalLost++;
		if (countGoalLost >= threshold) {
			returnGoalVal = 1;
		}
	} return returnGoalVal;
}

// Goal Tracking =============================================================================
double	intPanG = 0, dervPanG = 0, errorPanG = 0, preErrPanG = 0,
	PPanG = 0, IPanG = 0, DPanG = 0,
	intTiltG = 0, dervTiltG = 0, errorTiltG = 0, preErrTiltG = 0,
	PTiltG = 0, ITiltG = 0, DTiltG = 0,
	dtG = 0.04;
double	G_Pan_err_diff, G_Pan_err, G_Tilt_err_diff, G_Tilt_err, G_PanAngle, G_TiltAngle,
	pOffsetG, iOffsetG, dOffsetG,
	errorPanGRad, errorTiltGRad;
void trackGoal() {
	if (useVision) {
		if (receler.Goal_X != -1 && receler.Goal_Y != -1) { //printf("Tracking");
			//mode 1 ######################################################################
				//PID pan ==========================================================
				/*errorPanG  = (double)receler.Goal_X - (frame_X / 2);//160
				PPanG  = errorPanG  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

				intPanG += errorPanG * dtG;
				IPanG = intPanG * 0.0;

				dervPanG = (errorPanG - preErrPanG) / dtG;
				DPanG = dervPanG * 0.00001;

				preErrPanG = errorPanG;

				//posPan += PPanG*-1; //dikali -1 kalau receler terbalik dalam pemasangan
				posPan += (PPanG + IPanG + DPanG) * -1;


				//PID tilt ==========================================================
				errorTiltG = (double)receler.Goal_Y - (frame_Y / 2);//120
				PTiltG = errorTiltG * 0.00010; //Tune in Kp Tilt 0.00030

				intTiltG += errorTiltG * dtG;
				ITiltG = intTiltG * 0.0;

				dervTiltG = (errorTiltG - preErrTiltG) / dtG;
				DTiltG = dervTiltG * 0; //0.00001;

				preErrTiltG = errorTiltG;

				//posTilt += PTiltG;
				posTilt += (PTiltG + ITiltG + DTiltG);*/

			//mode 2 ######################################################################
				errorPanG  = (double)receler.Goal_X - (frame_X / 2);//160
				errorTiltG = (double)receler.Goal_Y - (frame_Y / 2);//120
				errorPanG *= -1;
				errorTiltG *= -1;
				errorPanG *= (77.32 / (double)frame_X); // pixel per angle
				errorTiltG *= (61.93 / (double)frame_Y); // pixel per angle

				errorPanGRad = (errorPanG * PI)/ 180;
				errorTiltGRad = (errorTiltG * PI) /180;
				//printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanG, errorTiltG);
				//printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanGRad, errorTiltGRad);
				//printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

				G_Pan_err_diff = errorPanGRad - G_Pan_err;
				G_Tilt_err_diff = errorTiltGRad - G_Tilt_err;

				// PID pan ==========================================================
				//PPanG  = G_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				PPanG  = G_Pan_err  * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				intPanG += G_Pan_err * dtG;
				IPanG = intPanG * 0.0;
				dervPanG = G_Pan_err_diff / dtG;
				//DPanG = dervPanG * kamera.panKD;
				DPanG = dervPanG * goal_panKD;
				G_Pan_err = errorPanGRad;
				posPan += (PPanG + IPanG + DPanG);

				// PID tilt ==========================================================
				//PTiltG = G_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
				PTiltG = G_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

				intTiltG += G_Tilt_err * dtG;
				ITiltG = intTiltG * 0.0;

				dervTiltG = G_Tilt_err_diff / dtG;
				//DTiltG = dervTiltG * kamera.tiltKD;
				DTiltG = dervTiltG * goal_tiltKD;

				preErrTiltG = errorTiltG;
				G_Tilt_err = errorTiltGRad;
				posTilt += (PTiltG + ITiltG + DTiltG) * -1;


			if      (posPan  >=  1.6)  { posPan  =  1.6; }
			else if (posPan  <= -1.6)  { posPan  = -1.6; }
			if      (posTilt <= -2.0)  { posTilt = -2.0; }
			else if (posTilt >= -0.4)  { posTilt = -0.4; }

			headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
			//GoalTracked = true;
		}

	}
}

// Follow Goal ===============================================================================
int	countReadyStop = 0;
void followGoal(double Xwalk, double SetPan, int mode){ //0 normal, 1 sambil belok
	errorfPan  = posPan - SetPan;

	if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && receler.Goal_X != -1 && receler.Goal_Y != -1) { //Stop
		countReadyStop++;
	} else { //Follow
		countReadyStop = 0;
	}

	if (countReadyStop >= 5) {
		PxMove = 0.00; //jalan ditempat
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.30; //0.045
	} else {
		PxMove = Xwalk; //0.08
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.35; //0.045
	}

	if (mode == 0) { // pake alfa
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
	else if (mode == 1) { // tanpa alfa
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
			Walk(PxMove, PyMove, 0.0);
		} else { 					//printf("DDDDDDDD\n");
			Walk(0.0, PyMove, 0.0);
		}
	}
}

// Body Tracking Goal ========================================================================
double  errorBodyPositionG,
        bodyP_ControllerG;
int     bodyTrueG = 0,
        delayTrueG = 0;
int bodyTrackingGoal(int threshold) {
//	trackGoal();

	errorBodyPositionG = 0 - headPan;
	bodyP_ControllerG = errorBodyPositionG * -0.5;//-0.5

	if (errorBodyPositionG >= -0.1 && errorBodyPositionG <= 0.1) {//untuk hasil hadap 0.8
		//motion("0");
		Walk(0.0, 0.0, 0.0);
		delayTrueG++;
	} else {
		trackGoal();
		//motion("9");

		bodyTrueG       =
		delayTrueG      = 0;

		if (bodyP_ControllerG < 0) { //kanan
			//Walk(rotateGoal_x, abs(rotateGoal_y), -abs(rotateGoal_a));
			Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
		} else { //kiri
			//Walk(rotateGoal_x, -abs(rotateGoal_y), abs(rotateGoal_a));
			Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
		}
	}

	if (delayTrueG >= threshold) {
		bodyTrueG = 1;
	} else {
		bodyTrueG = 0;
	}
	return bodyTrueG;
}

// Search Goal =====================================================================================================
double	goalPan = 0;
void saveGoalLocation() {
//	trackGoal();
	goalPan = headPan;
}

void saveSudutImu() {
//	sudut();
	saveAngle = angle;
}

int	prediksiGoalPan = 0;
double	Rotate = 0;
void prediksiArahGoal() {
	prediksiGoalPan = (int)(57.29 * headPan);
	Rotate = headPan * (8 / 1.57); //90 derajat = 8 detik //waktu rotate
}

int	count = 0;
bool	goalSearch = false,
	searchGoalFinish = false;
void panSearchGoal(double arah) { //printf("  panSearchBall\n\n");
	if (panRate < 0) { panRate = -panRate; }

	if (arah < 0) {
		headPan += panRate;
		if (headPan >= batasKiri) {
			prediksiGoalPan = 0;
			saveAngle = 0;
			Rotate = 0;
//			goalSearch = true;
		}
	} else {
		headPan -= panRate;
		if (headPan <= batasKanan) {
			prediksiGoalPan = 0;
			saveAngle = 0;
			Rotate = 0;
//			goalSearch = true;
		}
	}

	if	(headPan >= batasKiri)  { headPan = batasKiri; }
	else if	(headPan <= batasKanan) { headPan = batasKanan; }

	headMove(headPan, -2.0); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

double	predictGoalPan;
void predictGoal(double alpha, double tilt) {
	//predictGoalPan = (alpha * 1.57) / 90; //sudut * nilai per satu sudut (nilai servo)
	//predictGoalPan = alpha / (90 / 1.57); //sudut * nilai per satu sudut (nilai servo)
	predictGoalPan = alpha / 57.29; //sudut * nilai per satu sudut(nilai servo)

	if (predictGoalPan <= -1.6) { predictGoalPan = -1.6; }
	else if (predictGoalPan >= 1.6) { predictGoalPan = 1.6; }
	//if (predictGoalPan <= batasKanan) { predictGoalPan = batasKanan; }
	//else if (predictGoalPan >= batasKiri) { predictGoalPan = batasKiri; }
	//printf("  predict = %f\n\n",predictGoalPan);

	headMove(predictGoalPan, tilt);
}

int	goalSide = 0;
double	arahPandang = 0;
int cekArah() {
	arahPandang = angle - (57.29 * headPan);
	if (arahPandang >= -90 && arahPandang <= 90) { //gawang lawan
		goalSide = 0;
	} else { //gawang sendiri
		goalSide = 1;
	}
	return goalSide;
}



int	errorGoal = 0;
double	predictRotate;
void searchGoal(int mode) {
	if (goalSearch) { //printf("\n\ncari bola\n\n"); //kedua
		if (tunggu < 10) {
			loadBallLocation(0.0);
			elapsedTime = 0;
		        second = 0;
			reset = 0;
			robotDirection = false;
			timer = false;
			waiting = 0;
		} else if (tunggu > 50) {
			if (ballLost(20)) {
				if (waiting > 40) {
					resetCase1();
					stateCondition = 1;
				} else {
					panSearchBall(cSekarang);
					//tiltSearchBall(0.0);
					Walk(0.0, 0.0, 0.0);
				} waiting++;
			} else {
				trackBall();
				waiting = 0;

				if (mode == 1) { //rotate dgn trackgoal
					searchGoalFinish = true;
				} else if (mode == 2) { //rotate dgn timer
					if (reset > 5) {
						cekWaktu(abs(Rotate));
						if (timer) {
							Walk(0.0, 0.0, 0.0);
							searchGoalFinish = true;
						} else {
							if (Rotate < 0) { //kiri
								rotateParabolic(1, cSekarang);
							} else { //kanan
								rotateParabolic(-1, cSekarang);
							}
						}
					} else {
						setWaktu();
						reset++;
					}
				} else if (mode == 3) { //rotate dgn offset Imu
					if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
						Walk(0.0, 0.0, 0.0);
						searchGoalFinish = true;
					} else {
						if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
							if (reset > 5) {
								cekWaktu(20);
								if (timer) {
									Walk(0.0, 0.0, 0.0);
									searchGoalFinish = true;
								} else {
									if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30)) {
										if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
										else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
										if (errorGoal >= 20) { errorGoal = 20; }
									} else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30)) {
										if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
										else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
										if (errorGoal >= 20) { errorGoal = 20; }
									} else {
										errorGoal = 0;
										//if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
										//else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
										//if (errorGoal >= 20) { errorGoal = 20; }
									}

									if (useSideKick) {
										/*if (saveAngle >= 45) {
											if (((saveAngle - prediksiGoalPan) + errorGoal) <= 45) {
												modeKick = 4; //tendangSamping
												Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
											} else {
												modeKick = tendangJauh;
												Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
												lastDirection = angle;
											}
										} else if (saveAngle <= -45) {
											if (((saveAngle - prediksiGoalPan) + errorGoal) >= -45) {
												modeKick = 3; //tendangSamping
												Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
											} else {
												modeKick = tendangJauh;
												Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
												lastDirection = angle;
											}
										} else {
											if (((saveAngle - prediksiGoalPan) + errorGoal) >= 45) {
												modeKick = 3; //tendangSamping
												Imu((saveAngle - prediksiGoalPan) + errorGoal - 90, cSekarang);
											} else if (((saveAngle - prediksiGoalPan) + errorGoal) <= -45) {
												modeKick = 4; //tendangSamping
												Imu((saveAngle - prediksiGoalPan) + errorGoal + 90, cSekarang);
											} else {
												modeKick = tendangJauh;
												Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
												lastDirection = angle;
											}
										}*/

										if (prediksiGoalPan >= 45) { //kiri
											modeKick = 4; //tendangSamping
											Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										} else if (prediksiGoalPan <= -45) { //kanan
											modeKick = 3; //tendangSamping
											Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										} else {
											modeKick = tendangJauh;
											Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
											lastDirection = angle;
										}
									} else {
										modeKick = tendangJauh;
										Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										lastDirection = angle;
									}
								}
							} else {
								setWaktu();
								robotDirection = false;
								reset++;
							}
						} else {
							reset = 0;
							followBall(0);
						}
					}
				}
			}
		} else {
			elapsedTime = 0;
		        second = 0;
			reset = 0;
			robotDirection = false;
			timer = false;
			waiting = 0;

			if (ballLost(20)) {
				panSearchBall(cSekarang);
				//tiltSearchBall(0.0);
			} else {
				trackBall();
			} Walk(0.0, 0.0, 0.0);
		} tunggu++;
	} else { //pertama
		tunggu = 0;

		if (goalLost(20)) { //printf("\n\n  cari gawang\n\n");//fungsi yg cari gawang disini
			Walk(0.0, 0.0, 0.0);

			if (delay > 5) {
				cekWaktu(2.5);
				if (!timer) {
					if (second <= 2) {
						if (strategy.strategyNumber == 3 || strategy.strategyNumber == 4 || strategy.strategyNumber == 9 || strategy.strategyNumber == 10 || strategy.strategyNumber == 11 || strategy.strategyNumber == 12) {
							headMove(0.0, -2.0);
						} else {
							predictGoal(angle, -2.0);
						}
					} else if (second > 2 && second <= 2.5) {
						if (angle < 0) {
							headMove(-1.6, -2.0);
						} else {
							headMove(1.6, -2.0);
						}
					}
				} else {
					panSearchGoal(angle);
				}
			} else {
				setWaktu();
				count = 0;
				delay++;
			}

		} else { //printf("\n\n  rotate gawang\n\n");
			trackGoal();

			if (count > 50) { //50 //60
				if (((saveAngle - prediksiGoalPan) >= -90) && ((saveAngle - prediksiGoalPan) <= 90)) {
					if (mode == 1) {
						if (bodyTrueG == 1) {
							Walk(0.0, 0.0, 0.0);
							goalSearch = true;
						} else {
							bodyTrackingGoal(10); //20
						}
					} else if (mode == 2) {
						goalSearch = true;
					} else if (mode == 3) {
						goalSearch = true;
					}
				} else { //gawang sendiri
					Walk(0.0, 0.0, 0.0);

					prediksiGoalPan = 0;
					saveAngle = 0;
					Rotate = 0;

					goalSearch = true;
				}
			} else {
				Walk(0.0, 0.0, 0.0);

				prediksiArahGoal();
				saveSudutImu();
				//saveGoalLocation();

				count++;
			}
		}
	}
}

int	thisOffset = 0,
	headOffset = 0;
void searchGoal2(int mode) {
	if (goalSearch) { //2 (kedua)
		if (tunggu < 30) { //2.1 (liat bola)
			loadBallLocation(0.0);
			robotDirection = false;
			waiting = 0;

			reset = 0;
			elapsedTime = 0;
		        second = 0;
			timer = false;

			if (ballLost(20)) {
				panSearchBall(cSekarang);
			} else {
				trackBall();
			} Walk(0.0, 0.0, 0.0);
			tunggu++;
		} else { //2.2 (rotate Imu)
			if (ballLost(20)) {
				if (waiting > 40) {
					resetCase1();
					stateCondition = 1;
				} else {
					tiltSearchBall(0.0);
					Walk(0.0, 0.0, 0.0);
				} waiting++;
			} else {
				trackBall();
				waiting = 0;

				if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
					Walk(0.0, 0.0, 0.0);
					searchGoalFinish = true;
				} else {
					if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
						if (reset > 5) {
							cekWaktu(20);
							if (timer) {
								Walk(0.0, 0.0, 0.0);
								searchGoalFinish = true;
							} else {
								Imu(Rotate, cSekarang);
							}
						} else {
							setWaktu();
							robotDirection = false;
							reset++;
						}
					} else {
						reset = 0;
						followBall(0);
					}
				}
			}
		}
	} else { //1 (pertama)
		tunggu = 0;
		Walk(0.0, 0.0, 0.0);
		if (mode == 1) {
			if (goalLost(20)) { //1.1 (cari gawang)
				count = 0;
				if (delay > 5) {
					cekWaktu(0);
					if (second <= 2) { //tengah
						headMove(0.0, -2.0);
						headOffset = 0;			
					} else if (second > 2 && second <= 4) { //kanan
						headMove(-1.0, -2.0);
						headOffset = 45;			
					} else if (second > 4 && second <= 6) { //kiri
						headMove(1.0, -2.0);
						headOffset = -45;			
					} else { //tidak terdeteksi gawang
						Rotate = sudutTengah + estimasiAngle;;
						goalSearch = true;
					}
				} else {
					setWaktu();
					delay++;
				}
			} else { //1.2 (dapat gawang)

				if (count > 20) {
					Rotate = saveAngle + thisOffset;
					goalSearch = true;
				} else {
					saveSudutImu();

					if (receler.Goal_C == 0) { //kedua tiang terdeteksi
						if (receler.Goal_X >= 270 && receler.Goal_X <= 370) { //titik tengah gawang ada dibagian tengah
							//0
							thisOffset = 0 + headOffset;				
						} else {
							if (receler.Goal_X > 370) { //titik tengah gawang dibagian kanan
								//20
								thisOffset = 20 + headOffset;			
							} else { //titik tengah gawang dibagian kiri
								//-20
								thisOffset = -20 + headOffset;		
							}
						}
					} else if (receler.Goal_C > 0) { //hanya tiang kanan terdeteksi
						//-30+
						thisOffset = -30 + headOffset;				
					} else { //hanya tiang kiri terdeteksi
						//30+
						thisOffset = 30 + headOffset;					
					} count++;
				}
			}
		} else if (mode == 2) {
			if (goalLost(20)) { //1.1 (cari gawang)
				count = 0;
				if (delay > 5) {
					cekWaktu(0);
					if (second <= 1) { //tengah
						headMove(0.0, -2.0);
					} else if (second > 1 && second <= 2) { //kanan
						headMove(-1.0, -2.0);
					} else if (second > 2 && second <= 3) { //kiri
						headMove(1.0, -2.0);
					} else { //tidak terdeteksi gawang
						Rotate = 0;
						goalSearch = true;
					}
				} else {
					setWaktu();
					delay++;
				}
			} else { //1.2 (dapat gawang)
				trackGoal();

				if (count > 20) {
					Rotate = saveAngle - thisOffset;
					goalSearch = true;
				} else {
					prediksiArahGoal();
					saveSudutImu();

					if (receler.Goal_C == 0) { //kedua tiang terdeteksi
						if (receler.Goal_X >= 270 && receler.Goal_X <= 370) { //titik tengah gawang ada dibagian tengah
							//0
							thisOffset = 0 + prediksiGoalPan;
						} else {
							if (receler.Goal_X > 370) { //titik tengah gawang dibagian kanan
								//20
								thisOffset = 30 + prediksiGoalPan;
							} else { //titik tengah gawang dibagian kiri
								//-20
								thisOffset = -30 + prediksiGoalPan;
							}
						}
					} else if (receler.Goal_C > 0) { //hanya tiang kanan terdeteksi
						//30
						thisOffset = 30 + prediksiGoalPan;
					} else { //hanya tiang kiri terdeteksi
						//-30
						thisOffset = -30 + prediksiGoalPan;
					} count++;
				}
			}
		}
	}
}

bool	rotateGoal = false,
	ballposTracked = false;
void rotateToGoal(int mode) {
	if (!searchGoalFinish) { //pertama
		if (ballposTracked) { //printf("  searchgoal,"); //second
			searchGoal(3); //1 2 3
			//searchGoal2(2);
		} else { //printf("  positioning,"); //first
			if (ballLost(20)) { //20
				resetCase1();
				stateCondition = 1;
			} else {
				trackBall();
				if (mode == 1) {
					if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
//						saveBallLocation();
						ballposTracked = true;
					} else {
						followBall(0);
					}
				} else if (mode == 2) { //
					if (ballPos) { //positioning sebelum search goal
//						saveBallLocation();
						ballposTracked = true;
					} else {
						ballPositioning(0.0, cSekarang, ballPositioningSpeed); //0.15
					}
				}
			}
		}
	} else { //printf("  finish.........................................,"); //kedua
		//if (ballLost(20)) { //40
		//	resetCase1;
		//	stateCondition = 1;
		//} else {
			trackBall();
			rotateGoal = true;
		//}
	}
}

// Tendang ===================================================================================
bool	tendang = false;
void kick(int mode) {
//	trackBall();
	if (mode == 3 || mode == 4) {
		//if (angle < 0) { kanan = true; kiri = false; } //arah kanan
		//else { kiri = true; kanan = false; } //arah kiri
		if (mode == 3) { kanan = true; kiri = false; } //arah kanan
		else if (mode == 4) { kiri = true; kanan = false; } //arah kiri
	} else {
		if (posPan >= 0 && kanan == false && kiri == false) { //kiri
			kiri = true; kanan = false;
		} else if (posPan <= 0 && kanan == false && kiri == false) { //kanan
			kanan = true; kiri = false;
		}
	}

	if (kiri) { //kiri
	//if (posPan >= 0) { //kiri					
		if (ballPos) { //printf("ball pos left true\n");
			motion("0");
			//usleep(500000);
			//Walk(0.00, 0.00, 0.00);

			if (mode == 1 || mode == 2) {
				usleep(700000); //6
				motion("1");
			} else if (mode == 3 || mode == 4) {
				sleep(1); //10
				//motion("3");
				motion("4");
			} else if (mode == 5 || mode == 6) {
				usleep(700000);
				motion("5");
			}
			tendang = true;
		} else {
			ballPositioning(-pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
		}
	} if (kanan) { //kanan
	//} else { //kanan						
		if (ballPos) { //printf("ball pos right true\n");
			motion("0");
			//usleep(500000);
			//Walk(0.00, 0.00, 0.00);

			if (mode == 1 || mode == 2) {
				usleep(700000); //7
				motion("2");
			} else if (mode == 3 || mode == 4) {
				sleep(1); //10
				//motion("4");
				motion("3");
			} else if (mode == 5 || mode == 6) {
				usleep(700000); //7
				motion("6");
			}
			tendang = true;
		} else {
			ballPositioning(pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
		}
	}
}

void rotateKickOffImu(int sudut, int mode) {
	if (headTilt >= -1.4) {
		if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
			if (sudut >= 0) { //kiri
				if (ballPos) { //printf("ball pos left true\n");
					motion("0");
					//usleep(500000);
					//Walk(0.00, 0.00, 0.00);
					if (mode == 1 || mode == 2) {
						usleep(700000); //6
						motion("1");
					} else if (mode == 3 || mode == 4) {
						//usleep(1000000); //10
						sleep(1); //10
						//motion("3");
						motion("4");
					} else if (mode == 5 || mode == 6) {
						usleep(700000);
						motion("5");
					}
					tendang = true;
				} else {
					ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); //0.15
				}
			} else { //kanan
				if (ballPos) { //printf("ball pos right true\n");
					motion("0");
					//usleep(500000);
					//Walk(0.00, 0.00, 0.00);
					if (mode == 1 || mode == 2) {
						usleep(700000); //7
						motion("2");
					} else if (mode == 3 || mode == 4) {
						//usleep(1000000); //10
						sleep(1); //10
						//motion("4");
						motion("3");
					} else if (mode == 5 || mode == 6) {
						usleep(700000); //7
						motion("6");
					}
					tendang = true;
				} else {
					ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); //0.15
				}
			}
		} else {
			if (headTilt >= (cAktif + 0.2) && headPan >= -0.4 && headPan <= 0.4) {
				Imu(sudut, cSekarang - 0.20);
			} else {
				robotDirection = false;
				followBall(0);
			}
		}
	} else {
		if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
		else if	(posTilt < -2.0) { posTilt = -2.0; }

		errorfPan  = posPan - SetPointPan;
		errorfTilt = posTilt - SetPointTilt;

		if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && receler.Ball_X != -1 && receler.Ball_Y != -1) { //Stop(bola sudah dekat)
			PxMove = 0.0; //jalan ditempat
			PyMove = errorfPan * 0.040; //0.045
			PaMove = errorfPan * 0.20; //0.30; //0.045
		} else { //Kejar Bola(bola masih jauh)
			PxMove = kejarMax; //0.06
			PyMove = errorfPan * 0.045; //0.045
			PaMove = errorfPan * 0.25; //0.35; //0.045
		}

		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
		//followBall(0);
	}
}

void rotateKickOff(double timeRotate, int mode) {
//	trackBall();

	if (headTilt >= -1.0) {
	//if (headTilt >= -0.8 && headPan >= -0.4 && headPan <= 0.4) {
		if (reset > 5) { //printf("set......................!!!\n");
			cekWaktu(abs(timeRotate));
			if (timer) { //printf("true......................!!!\n");
				if (posPan >= 0) { //kiri
					if (ballPos) { //printf("ball pos left true\n");
						motion("0");
						//usleep(500000);
						//Walk(0.00, 0.00, 0.00);

						if (mode == 1 || mode == 2) {
							usleep(700000); //6
							motion("1");
						} else if (mode == 3 || mode == 4) {
							//usleep(1000000); //10
							sleep(1); //10
							//motion("3");
							motion("4");
						} else if (mode == 5 || mode == 6) {
							usleep(700000);
							motion("5");
						}
						tendang = true;
					} else {
						ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); //0.15
					}
				} else { //kanan
					if (ballPos) { //printf("ball pos right true\n");
						motion("0");
						//usleep(500000);
						//Walk(0.00, 0.00, 0.00);

						if (mode == 1 || mode == 2) {
							usleep(700000); //7
							motion("2");
						} else if (mode == 3 || mode == 4) {
							//usleep(1000000); //10
							sleep(1); //10
							//motion("4");
							motion("3");
						} else if (mode == 5 || mode == 6) {
							usleep(700000); //7
							motion("6");
						}
						tendang = true;
					} else {
						ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); //0.15
					}
				}
			} else {
				motion("9");
				//rotateParabolic(timeRotate, cSekarang);
				if (timeRotate < 0) { //kanan
					Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
					//Walk(-0.010, rotateGoal_y, -rotateGoal_a);
				} else { //kiri
					Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
					//Walk(-0.005, -0.029, rotateGoal_a);
				}
			}
		} else { //printf("cek......................!!!\n");
			motion("9");
			Walk(0.0, 0.0, 0.0);
			setWaktu();
			reset++;
		}
	} else {
		reset = 0;

		/*if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
		else if	(posTilt < -2.0) { posTilt = -2.0; }

		errorfPan  = posPan - SetPointPan;
		errorfTilt = posTilt - SetPointTilt;

		if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && receler.Ball_X != -1 && receler.Ball_Y != -1) { //Stop(bola sudah dekat)
			PxMove = 0.0; //jalan ditempat
			PyMove = errorfPan * 0.040; //0.045
			PaMove = errorfPan * 0.20; //0.30; //0.045
		} else { //Kejar Bola(bola masih jauh)
			PxMove = lari; //0.06
			PyMove = errorfPan * 0.045; //0.045
			PaMove = errorfPan * 0.25; //0.35; //0.045
		}

		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}*/
		followBall(0);
	}
}

// Localization ==============================================================================
int	kecepatanRobot = 25, //cm
	finalGoalValue = 300; //jarak berhenti robot
int	jarakTiangKanan = 0,
	jarakTiangKiri = 0,
	goalDistanceValue = 0;
double	Timing = 0;
int kalkulasiJarakGawang(int tiangKiri, int tiangKanan) {
	jarakTiangKiri = tiangKiri;
	jarakTiangKanan = tiangKanan;

	if (jarakTiangKiri > jarakTiangKanan) {
		if (jarakTiangKiri > 900) {
			goalDistanceValue = jarakTiangKanan;
		} else {
			goalDistanceValue = jarakTiangKiri;
		}
	} else {
		if (jarakTiangKanan > 900) {
			goalDistanceValue = jarakTiangKiri;
		} else {
			goalDistanceValue = jarakTiangKanan;
		}
	}

	if (jarakTiangKiri == -1 && jarakTiangKanan == -1) {
		Timing = 0;
	} else {
		Timing = (goalDistanceValue - finalGoalValue) / kecepatanRobot;
	}

	return Timing;
}

//////////////////////////////////////////////////////////////////////////////////////////////
double 	targetCoorY, sekarangCoorY, Ytar,
	targetCoorX, sekarangCoorX, Xtar,
	targetCoorR, sekarangCoorR, setPointRotateR1, setPointRotateR2,
	setfPointY,
	setRotatePointR,
	errorfX,
	errorfY,
	errorAngle,
	LyMove = 0,	//Y
	LxMove = 0,	//X
	LaMove = 0;	//A
int	cL,	//count lock for atribut target
	cD,	//count peralihan dari rotate ke jalan
	cR,	//count peralihan dari break ke rotate
	cB,	//count untuk break ketika sudah didalam setpoint target
	cZ,
	cA,
	cV,cY,
	cN;
bool	donePosition = false,		//var untuk menandakan fungsi selesai
	kembaliMasuk = false,		//var untuk menandakan target diantara >90 && <-90
	awalMasuk = false,		//var untuk menandakan target diantara <90 && >-90
	cX = false,			//var untuk menandakan koordinat x sudah dalam range
	lockMidBack = false,		//var untuk mengunci posisi balik yang dituju -> tengah (main strategi)
	lockLeftBack = false,		//var untuk mengunci posisi balik yang dituju -> kiri (support strategi)
	lockRightBack = false,		//var untuk mengunci posisi balik yang dituju -> kanan (support strategi)
	balikTengah = false,		//var untuk lock posisi balik use koordinasi
	balikSampingKanan = false,	//var untuk lock posisi balik use koordinasi
	balikSampingKiri = false;	//var untuk lock posisi balik use koordinasi
//////////////////////////////////////////////////////////////////////////////////////////////

int	tama = 0;
double	Waktu = 0;
void updateCoordinatFromVision() {
	if (receler.RobotCoor_X >=-450 && receler.RobotCoor_X <=450 && receler.RobotCoor_Y >= -300 && receler.RobotCoor_Y<=300) {
		if (receler.RobotCoor_X!=-1 && receler.RobotCoor_Y!=-1) {
			initialPos_X = receler.RobotCoor_X;
			initialPos_Y = receler.RobotCoor_Y;
			odometry.deltaPos_X = odometry.deltaPos_Y = 0;
		}
	}
}

void koordinasiRobotBalik(){ // robotStatus, jika 0 = kill jika 1 = run
	if (robotID == 1) {
		//printf("   this is Robot 1..........................\n");
		if (robotStatus == 1) {	//run
			balikTengah = true;	//eksekusi
			balikSampingKanan = false;
			balikSampingKiri = false;
			//printf("   Robot 1 Run...........................\n");
		} else {		//kill
			balikTengah = false;
			balikSampingKanan = false;
			balikSampingKiri = false;
			//printf("   Robot 1 Kill..........................\n");
		}
	} else if (robotID == 3) {
		//printf("   this is Robot 3..........................\n");
		if (robotStatus == 1) {	//run
			if (koordinasi.robot1Status == 0) {		//ketika robot 1 tidak aktif
				balikTengah = true;
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 3 Run Balik Tengah..........................\n");
			} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
				balikTengah = false;
				balikSampingKanan = true;
				balikSampingKiri = false;
				//printf("   Robot 3 Run Balik Samping Kanan..........................\n");
			}
		} else {		//kill
			balikTengah = false;
			balikSampingKanan = false;
			balikSampingKiri = false;
			//printf("   Robot 3 Kill..........................\n");
		}
	} else if (robotID == 4) {
		//printf("   this is Robot 4..........................\n");
		if (robotStatus ==1 ) {	//run
			if (koordinasi.robot1Status == 0) {	//ketika robot 1 tidak aktif
				if (koordinasi.robot3Status == 0) {	//ketika robot 3 tidak aktif
					balikTengah = true;
					balikSampingKanan = false;
					balikSampingKiri = false;
					//printf("   Robot 4 Run Balik Tengah..........................\n");
				} else if (koordinasi.robot3Status == 1) {	//ketika robot 3 aktif
					balikTengah = false;
					balikSampingKanan = true;
					balikSampingKiri = false;
					//printf("   Robot 4 Run Balik Samping Kanan..........................\n");
				}
			} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
				if (koordinasi.robot3Status == 0) {	//ketika robot 3 tidak aktif
					balikTengah = false;
					balikSampingKanan = true;
					balikSampingKiri = false;
					//printf("   Robot 4 Run Balik Samping Kanan..........................\n");
				} else if (koordinasi.robot3Status == 1) {	//ketika robot 3 aktif
					balikTengah = false;
					balikSampingKanan = false;
					balikSampingKiri = true;
					//printf("   Robot 4 Run Balik Samping Kiri..........................\n");
				}
			}
		} else {		//kill
			balikTengah = false;
			balikSampingKanan = false;
			balikSampingKiri = false;
			//printf("   Robot 4 Kill..........................\n");
		}
	} else if (robotID == 5) {
		//printf("   this is Robot 5..........................\n");
		if (robotStatus == 1) {	//Run
			if (koordinasi.robot1Status == 0) {	//ketika robot 1 tidak aktif
				if (koordinasi.robot3Status == 0 || koordinasi.robot4Status == 0) {	//ketika robot 3 tidak aktif atau robot 4 tidak aktif
					balikTengah = true;
					balikSampingKanan = false;
					balikSampingKiri = false;
					//printf("   Robot 5 Run Balik Samping Tengah..........................\n");
				} else if (koordinasi.robot3Status == 1 || koordinasi.robot4Status == 1) {	//ketiak robot 3 aktif atau robot 4 akti
					balikTengah = false;
					balikSampingKanan = true;
					balikSampingKiri = false;
					//printf("   Robot 5 Run Balik Samping Kanan..........................\n");
				}
			} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
				if (koordinasi.robot3Status == 0 || koordinasi.robot4Status == 0) {	//ketika robot 3 tidak aktif atau robot 4 tidak aktif
					balikTengah = false;
					balikSampingKanan = true;
					balikSampingKiri = false;
					//printf("   Robot 5 Run Balik Samping Kanan..........................\n");
				} else if (koordinasi.robot3Status == 1 || koordinasi.robot4Status == 1) {	//ketika robot 3 aktif atau robot 4 aktif
					balikTengah = false;
					balikSampingKanan = false;
					balikSampingKiri = true;
					//printf("   Robot 5 Run Balik Samping Kiri..........................\n");
				}
			}
		} else {
			balikTengah = false;
			balikSampingKanan = false;
			balikSampingKiri = false;
			//printf("   Robot 5 Kill..........................\n");
		}
	}
}

void moveLokalisasi( double mode, double Xtarget, double Ytarget) {
	/*Robot berjalan berdasarkan koordinat yang di imput*/
	Xtar = Xtarget; Ytar = Ytarget;
	targetCoorX = robotPos_X - Xtarget;//-240--240	= 0
	targetCoorY = robotPos_Y - Ytarget;//-320--0	= -320
	targetCoorR = sqrt((targetCoorX*targetCoorX)+(targetCoorY*targetCoorY));
	if (robotPos_X > Xtarget) {		//Target ada dibelakang
		if (robotPos_Y > Ytarget) {	//saat ini sebelah kanan
			setRotatePointR = -180 + asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 180 + asin(targetCoorY/targetCoorR)*(180/PI);
		}
	} else if (robotPos_X < Xtarget) {	//Target ada didepan
		if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y > Ytarget) {	//saat ini sebalah kanan
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		}
	}

	if(robotPos_X < (Xtarget+15) && robotPos_X > (Xtarget-15)){	//Koreksi koordinatX
		cX = true;
	} else {
		cB = 0;
	}
	if(cX){
		if (mode == 0) { //Koreksi koordinat Y Jika koordinat X sudah masuk range
			if (robotPos_Y < (Ytarget+15) && robotPos_Y > (Ytarget-15)) {
					cB ++;
			} else {
				if (robotPos_Y > (Ytarget+15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,-90);
						} else {
							rotateBodyImuNew(-90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				} else if (robotPos_Y < (Ytarget-15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,90);
						} else {
							rotateBodyImuNew(90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				}
			}
		} else if (mode == 1) {	//tanpa koreksi koordinat Y
			cB ++;
		}
	}

	if(cB >= 1){
		donePosition = true;
		posRotateNew = false;
		LxMove = 0.0;
	} else {
		if(!cX){
			LxMove = kejarMax;
			jalanDirection(LxMove,0.0,setRotatePointR);	//Jika bisa baca gawang
		}
	}
}

void moveBackLokalisasi( double mode, double Xtarget, double Ytarget) {
	/*Robot berjalan berdasarkan koordinat yang di imput*/
	Xtar = Xtarget; Ytar = Ytarget;
	targetCoorX = robotPos_X - Xtarget;//-240--240	= 0
	targetCoorY = robotPos_Y - Ytarget;//-320--0	= -320
	targetCoorR = sqrt((targetCoorX*targetCoorX)+(targetCoorY*targetCoorY));
	if (robotPos_X > Xtarget) {		//Target ada dibelakang
		if (robotPos_Y > Ytarget) {	//saat ini sebelah kanan
			setRotatePointR = -180 + asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 180 + asin(targetCoorY/targetCoorR)*(180/PI);
		}
	} else if (robotPos_X < Xtarget) {	//Target ada didepan
		if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y > Ytarget) {	//saat ini sebalah kanan
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		}
	}

	if(robotPos_X < (Xtarget+15) && robotPos_X > (Xtarget-15)){	//Koreksi koordinatX
		cX = true;
	} else {
		cB = 0;
	}
	if(cX){
		if (mode == 0) { //Koreksi koordinat Y Jika koordinat X sudah masuk range
			if (robotPos_Y < (Ytarget+15) && robotPos_Y > (Ytarget-15)) {
					cB ++;
			} else {
				if (robotPos_Y > (Ytarget+15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,-90);
						} else {
							rotateBodyImuNew(-90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				} else if (robotPos_Y < (Ytarget-15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,90);
						} else {
							rotateBodyImuNew(90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				}
			}
		} else if (mode == 1) {	//tanpa koreksi koordinat Y
			cB ++;
		}
	}

	if(cB >= 1){
		donePosition = true;
		posRotateNew = false;
		LxMove = 0.0;
	} else {
		if(!cX){
			LxMove = kejarMax;
			//jalanDirection(LxMove,0.0,setRotatePointR);	//Jika bisa baca gawang
			jalanDirection(LxMove, 0.0, 180);		//Jika tidak bisa baca gawang
		}
	}
}

void localization() {
	if (reset > 5) {
		cekWaktuLokalisasi(30);
		//lock initial pos
		//if (lastSecRemaining == 0) { //jika data game controller tidak terbaca
		if (gameController.Remaining == 0) { //jika data game controller tidak terbaca
			motion("0");
			resetAllVariable();
			predictGoal(angle, posTiltGoal);
			backPosition = false;
		//} else if (lastSecRemaining == 600) { //masuk lapangan pertama kali mulai permainan
		} else if ((gameController.Remaining == 600 && gameController.SecondaryState == 0) || (gameController.Remaining == 300 && gameController.SecondaryState == 2) || switched) { //masuk lapangan pertama kali mulai permainan
			if(awalMasuk){	//ketika sudah sampai posisi awal masuk
				refreshMoveLokalisasi();
				if (goalLost(20)) {
					predictGoal(angle, posTiltGoal);
				} else {
					trackGoal();
					if(useUpdateCoordinate) { updateCoordinatFromVision(); }		
				}
			} else {		//ketika belum sampai posisi masuk
				if (donePosition && posRotateNew) {		//ketika sudah sampai tujuan dan sudah rotate
					motion ("0");
					predictGoal(angle, posTiltGoal);
					count = 0;
					awalMasuk = true;
				} else if ( donePosition && !posRotateNew) {		//ketika sudah sampai tujuan tapi belum rotate
					headMove(0.0, posTiltGoal);
					if (cR >= 10) {	//counting terpenuhi
						if (strategy.strategyNumber <= 4 || strategy.strategyNumber == 13) {		//jika tombol strategy <= 4 atau == 13 arah rotate 0 derajat
							rotateBodyImuNew(0);
						} else {		// selain strategy <=4 & 13, arah rotate menuju titik tengah lapangan
							rotateBodyImuNew(odometry.ArchSinTeng);
						}
					} else {	//counting belum terpenuhi untuk rotate
						Walk (0.0, 0.0, 0.0);
						cR ++ ;
					}
				} else {		//mulai bergerak
					motion ("9");
					if (odometry.walkTot >= 6){	//counting jalan 6 langkah terpenuhi
						searchBallRectang(-1.6, -1.6, -0.8, 1.6);
						if (strategy.strategyNumber <=4) {
							moveLokalisasi (0, -85, 0);
						} else if (strategy.strategyNumber == 13){
							moveLokalisasi (0, -150, 0);
						} else {
							if (robotPos_Y >= 0) {
								if (strategy.strategyNumber == 14) {
									moveLokalisasi (0, -240, 100);
								} else {
									moveLokalisasi (0, -200, 100);
								}
							} else if (robotPos_Y < 0) {
								if (strategy.strategyNumber == 14) {
									moveLokalisasi (0, -240, -100);
								} else {
									moveLokalisasi (0, -200, -100);
								}
							}
						}
					} else {		//sini dulu, sampai counting jalan terpenuhi
						predictGoal(angle, -1.6);
						if (count >5) {		//counting biasa terpenuhi
							Walk(kejarMax, 0.0, 0.0);
						} else {		///sini, counting dulu
							count ++;
							Walk(0.0, 0.0, 0.0);
						}
					}
				}
			} backPosition = false;
		//} else if (lastSecRemaining != 600) { //kembali ke posisi setelah ada goal/dropball
		} else if ((gameController.Remaining != 600 && gameController.SecondaryState == 0) || (gameController.Remaining != 300 && gameController.SecondaryState == 2) || !switched) { //kembali ke posisi setelah ada goal/dropball
			//printf("\n  >>>>>>>> balik ke belakang, goal/dropball <<<<<<<< \n");
			if(kembaliMasuk){
				refreshMoveLokalisasi();
			} else {
				if (useCoordination){		//kondisi balik ketika menggunakan useCoordination
					if (donePosition && posRotateNew) {
						motion("0");
						predictGoal(angle, posTiltGoal);
						count = 0;
						kembaliMasuk = true;
					} else if (donePosition && !posRotateNew) {
						headMove(0.0, posTiltGoal);
						if (cR >= 10){
							if (balikTengah){
								rotateBodyImuNew(0);
							} else if (balikSampingKanan || balikSampingKiri) {
								rotateBodyImuNew(odometry.ArchSinTeng);
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							cR++;
						}
					} else {
						motion("9");
						if (count>5) {
							if (balikTengah) {
								if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //ketika dikiri dan sebagai attack
								//if (gameController.KickOff == TEAM || gameController.KickOff == 128) { //ketika dikiri dan sebagai attack
									moveLokalisasi(0, -85, 0); //printf("Serang...\n");
								} else { //ketika dikiri dan sebagai defense
									moveLokalisasi(0, -150, 0); //printf("Defense...\n");
								}
							} else if (balikSampingKanan) {
								moveLokalisasi(0, -200, 130);
							} else if (balikSampingKiri) {
								moveLokalisasi(0, -200, -130);
							}

							//plan 2	tidak update tvision tanpa koordinasi tapi follow goal (gawang tim jelek)
							/*
							if (sebentar > 30) {
								if (angle < -90 || angle > 90) {
									if (goalLost(20)) {
										panSearchBall(posTiltLocal);
										moveBackLokalisasi(0, -150, 0);
									} else {
										trackGoal();
										if ( robotPos_X <= -150 ) {
											donePosition = true;
											posRotateNew = false;
										} else {
											donePosition = false;
											followGoal(kejarMax, 0.0, 0);
										}
									}
								} else {
									headMove(0.0, -1.9);
									moveBackLokalisasi(0, -150, 0);
								}
							} else {
								headMove(0.0, -1.9);
								moveBackLokalisasi(0, -150, 0);
								sebentar++;
							}
							*/
							if (cX){
								headMove(0.0, -1.2);
							} else {
								if (goalLost(20)) {
									panSearchBall(posTiltLocal);
								} else {
									trackGoal();
									if(useUpdateCoordinate) { updateCoordinatFromVision(); }										
								}
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							count ++;
						}
					}
				} else {	//tanpa koordinasi
					if (donePosition && posRotateNew) {
						motion("0");
						predictGoal(angle, posTiltGoal);
						count = 0;
					} else if (donePosition && !posRotateNew) {
						predictGoal (angle, posTiltGoal);
						if (cR >= 10) {
							if (strategy.strategyNumber <= 4 || strategy.strategyNumber == 13) {
								rotateBodyImuNew(0);
							} else {
								rotateBodyImuNew(odometry.ArchSinTeng);
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							cR ++;
						}
					} else {
						if ( count >= 5) {
							if (strategy.strategyNumber <=4 || strategy.strategyNumber == 13) {
								lockMidBack = true;
								if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //ketika dikiri dan sebagai attack
								//if (gameController.KickOff == TEAM || gameController.KickOff == 128) { //ketika dikiri dan sebagai attack
									moveLokalisasi(0, -85, 0); //printf("Serang...\n");
								} else { //ketika dikiri dan sebagai defense
									moveLokalisasi(0, -150, 0); //printf("Defense...\n");
								}
							} else {
								if (robotPos_Y >= 0 && !lockRightBack && !lockLeftBack ){
									lockRightBack = true;//moveLokalisasi(1,-200,120);
									lockLeftBack = false;
								} else if (robotPos_Y < 0 && !lockRightBack && !lockLeftBack) {
									lockLeftBack = true;//moveLokalisasi(1,-200,-120);
									lockRightBack = false;
								}
								
								if (lockRightBack) {
									moveLokalisasi(0, -200, 50);
								}

								if (lockLeftBack) {
									moveLokalisasi(0, -200, -50);
								}
							}

							//plan 2	tidak update tvision tanpa koordinasi tapi follow goal (gawang tim jelek)
							/*
							if (sebentar > 30) {
								if (angle < -90 || angle > 90) {
									if (goalLost(20)) {
										panSearchBall(posTiltLocal);
										moveBackLokalisasi(0, -150, 0);
									} else {
										trackGoal();
										if ( robotPos_X <= -150 ) {
											donePosition = true;
											posRotateNew = false;
										} else {
											donePosition = false;
											followGoal(kejarMax, 0.0, 0);
										}
									}
								} else {
									headMove(0.0, -1.9);
									moveBackLokalisasi(0, -150, 0);
								}
							} else {
								headMove(0.0, -1.9);
								moveBackLokalisasi(0, -150, 0);
								sebentar++;
							}
							*/

							if (cX){
								headMove(0.0, -1.2);
							} else {
								if (goalLost(20)) {
									panSearchBall(posTiltLocal);
								} else {
									trackGoal();
									if(useUpdateCoordinate) { updateCoordinatFromVision(); }										
								}
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							count ++;
						}
					}
				}
			} backPosition = true;
		}
	} else {
		setWaktuLokalisasi();
		refreshMoveLokalisasi();
		lockMidBack = false;
		lockLeftBack = false;
		lockRightBack = false;
		kembaliMasuk = false;
		awalMasuk = false;
		sebentar = 0;
		koordinasiRobotBalik();
		reset++;
	}
}

void displayMoveLokalisasi(){
//	printf(" Arah Pandang = %.f , Goal Side = %d\n", arahPandang, goalSide);
	printf("\n  stateGameController = %d,", stateGameController);
	printf(" Robot Bergerak = %d\n", odometry.robotBergerak);
	printf(" Walk X = %.2lf, Y = %.2lf, A = %.2lf\n", odometry.RobotWalk_X, odometry.RobotWalk_Y, odometry.RobotWalk_A);
	printf(" Sudut = %.f\n", angle);
	if(walkX < 0){
		printf(" Jumlah Mundur = %d\n", odometry.walkTotMun);
	} else if (walkX > 0){
		printf(" Jumlah Maju = %d\n", odometry.walkTot);
	}
	if(walkY > 0) {
		printf(" Jumlah Kiri = %d\n", odometry.csmKiri);
	} else if(walkY < 0) {
		printf(" Jumlah Kanan = %d\n", odometry.csmKanan);
	}
//	printf(" Jarak gawang = %d\n",receler.Goal_LD);
	//printf(" Initial Pos X = %.2lf, Y = %.2lf\n ", initialPos_X, initialPos_Y);
	printf(" Odometry  X = %.2lf, Y = %.2lf\n ", odometry.deltaPos_X, odometry.deltaPos_Y);
	//printf(" POSISI SEKARANG X = %.2lf , Y = %.2lf\n", robotPos_X, robotPos_Y);
//	printf(" POSISI TARGET   X = %.2lf , Y = %.2lf\n", Xtar, Ytar);
//	printf(" Angle = %.f , AngleTar = %.f\n", angle, setRotatePointR);
//	printf(" SP1 = %.f , SP2 = %.f\n", setPoint1, setPoint2);
//	printf(" Done Position  = %d\n", donePosition);
//	printf(" CountLock	= %d\n", cL);
//	printf(" CountBreak	= %d\n",cB);
//	printf(" CountRotate	= %d\n",cR);
//	printf(" CountDiam	= %d\n",cD);
//	printf(" CZ = %d , CX = %d , CV = %d , CN = %d\n", cZ, cX, cV, cN);
	printf("\n");
}

/*void lokalisasiTimer() {
	if (reset > 5) {
		cekWaktuLokalisasi(30);

		//if (lastSecRemaining == 0) { //jika data game controller tidak terbaca
		if (gameController.Remaining == 0) { //jika data game controller tidak terbaca
			motion("0");
			resetAllVariable();
			predictGoal(angle, -1.6);
			backPosition = false;
		//} else if (lastSecRemaining == 600) { //masuk lapangan pertama kali mulai permainan
		} else if (gameController.Remaining == 600 || switched) { //masuk lapangan pertama kali mulai permainan
			if (strategy.strategyNumber <= 4 || strategy.strategyNumber == 13) { //masuk posisi tengah
				//printf("\n  >>>>>>>> awal, masuk posisi tengah <<<<<<<< \n");
				tama = 30-13;
			} else { //masuk posisi samping
				//printf("\n  >>>>>>>> awal, masuk posisi samping <<<<<<<< \n");
				tama = 30-20;
			}

			if (second2 <= 5) { //awal
				motion("9");
				predictGoal(angle, -1.6);
				Walk(kejar, 0.0, 0.0);
			} else if (second2 > 5 && second2 <= tama) { //tengah
				if (second2 >= (tama-3)) {
					headMove(0.0, posTiltGoal);
				} else {
					searchBallRectang(-1.6, -1.6, -0.8, 1.6);
				} Walk(kejar, 0.0, 0.0);
				posRotate = GoalTracked = false;
			} else { //akhir
				if (posRotate || GoalTracked) {
					if (goalLost(20)) {
						predictGoal(angle, posTiltGoal);
						Walk(0.0, 0.0, 0.0);
					} else {
						trackGoal();
						if (headPan >= -0.1 && headPan <= 0.1) {
							motion("0");
						} else {
							motion("9");
							followGoal(0.0, 0.0, 0);
						}
					}
				} else {
					if (goalLost(20)) {
						headMove(0.0, posTiltGoal);
						rotateBodyImu(0);
					} else {
						trackGoal();
						Walk(0.0, 0.0, 0.0);
					}
				}
			} backPosition = false;
		//} else if (lastSecRemaining != 600) { //kembali ke posisi setelah ada goal/dropball
		} else if (gameController.Remaining != 600 || !switched) { //kembali ke posisi setelah ada goal/dropball
			//printf("\n  >>>>>>>> balik ke belakang, goal/dropball <<<<<<<< \n");

			if (second2 < 4) { //awal
				motion("9");
				headMove(0.0, posTiltGoal);
				jalanDirection(kejar, 0.0, 180);
			} else if (second2 >= 4 && second2 < 8) {
				if (goalLost(20)) {
					if (second2 < 5) {
						headMove(0.0, -1.9);
					} else if (second2 > 6) {
						headMove(1.0, -1.9);
					} else {
						headMove(-1.0, -1.9);
					} Walk(0.0, 0.0, 0.0);
					//searchBallPan(0.8, -1.9);
					//jalanDirection(0.0, 0.0, 180);
				} else {
					trackGoal();
					prediksiArahGoal();
					//if ((angle - prediksiGoalPan) < -90 || (angle - prediksiGoalPan) > 90) {
						Walk(0.0, 0.0, 0.0);
						//followGoal(0.0, 0.0, 0);
						Waktu = kalkulasiJarakGawang(receler.Goal_LD, receler.Goal_RD); //jarak menjadi waktu
						if (Waktu > 16) { Waktu = 16; }
						else if(Waktu <= 0) { Waktu = 16; }
					//} else {
					//	jalanDirection(kejarMax, 0.0, 180);
					//}
				}
			} else if (second2 >= 8  && second2 < 8+Waktu) { //followGoal
				if (goalLost(20)) {
					panSearchBall(-1.9);
					jalanDirection(kejarMax, 0.0, 180);
				} else {
					trackGoal();
					if (headPan >= -0.2 && headPan <= 0.2) {
						Walk(kejarMax, 0.0, 0.0);
					} else {
						followGoal(kejarMax, 0.0, 0);
					}
				}
			} else if (second2 >= 8+Waktu  && second2 < 12+Waktu) { //tengah
				predictGoal(angle, posTiltGoal);
				jalanDirection(0.0, 0.0, 0);
				posRotate = GoalTracked = false;
			} else if (second2 >= 12+Waktu) { //akhir
				if (posRotate || GoalTracked) {
					if (goalLost(20)) {
						panSearchBall(posTiltGoal);
						Walk(0.0, 0.0, 0.0);
					} else {
						trackGoal();
						if (headPan >= -0.1 && headPan <= 0.1) {
							motion("0");
						} else {
							motion("9");
							followGoal(0.0, 0.0, 0);
						}
					}
				} else {
					if (goalLost(20)) {
						if (angle >= -15 && angle <= 15) {
							panSearchBall(posTiltGoal);
						} else {
							headMove(0.0, posTiltGoal);
						} rotateBodyImu(0);
					} else {
						trackGoal();
						Walk(0.0, 0.0, 0.0);
					}
				}
			} backPosition = true;
		}
	} else {
		setWaktuLokalisasi();
		reset++;
	}
}*/

/*void sendLokalisasi(double X, double Y, double A, int Ld, int Rd, int Gc, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy, double odomImu) {
        if (Ld >= 0 && Ld <= 900 && Rd >= 0 && Rd <= 900 && goalSide == 0) { //di dalam lapangan dan yang dideteksi adalah gawang lawan
                if (abs(Ld-Rd) <= 260 && Gc == 0) { // jarak selisih antar tiang
                //if (abs(receler.Goal_LH-receler.Goal_RH) <= 50) {
                        sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%.f,%.f,%.f,%.f", X, Y, A, Ld, Rd, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
                        sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
                } else {
                        sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%.f,%.f,%.f,%.f", X, Y, A, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
                        sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
                }
        } else {
                sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%.f,%.f,%.f,%.f", X, Y, A, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
                sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
        }
      // printf("  dataLokalisasi = %s\n", dataLokalisasi);
}*/

void sendLokalisasi(int goal, int Ld, int Rd, int Gc, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy, double odomImu) {
	if ( receler.Goal_X >= 260 && receler.Goal_X <= 380 && receler.Goal_Y >= 210 && receler.Goal_Y <= 270 ) {
		if (Ld >= 0 && Ld <= 900 && Rd >= 0 && Rd <= 900){
			if (abs(Ld-Rd) <= 260 && Gc == 0){
				sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, Ld, Rd, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
				sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
			} else {
				sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
				sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
			}
		} else {
			sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
			sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
		}
	} else {
		sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
		sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
	}
       //printf("  dataLokalisasi = %s\n", dataLokalisasi);
}


void * sendData(void * argument) { //send socket lokalisasi
        while (1) {
			//sleep(1);
			usleep(500000);
			sendLokalisasi(goalSide, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, 360-sendAngle, receler.Ball_D, 57.29*headPan, robotPos_X, robotPos_Y, odometry.ArchSin);
        }
}
/*void sendLokalisasi(double X, double Y, double A, int Ld, int Rd, int Gc, int setPointImu, int imu, int Bd, double sudutPan) {
        if (Ld >= 0 && Ld <= 900 && Rd >= 0 && Rd <= 900 && goalSide == 0) { //di dalam lapangan dan yang dideteksi adalah gawang lawan
                if (abs(Ld-Rd) <= 260 && Gc == 0) { // jarak selisih antar tiang
                //if (abs(receler.Goal_LH-receler.Goal_RH) <= 50) {
                        sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.f", X, Y, A, Ld, Rd, setPointImu, imu, Bd, sudutPan);
                        sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
                } else {
                        sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.f", X, Y, A, -1, -1, setPointImu, imu, Bd, sudutPan);
                        sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
                }
        } else {
                sprintf(dataLokalisasi, "%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.f", X, Y, A, -1, -1, setPointImu, imu, Bd, sudutPan);
                sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
        }
        //printf("  dataLokalisasi = %s\n", dataLokalisasi);
}

void * sendData(void * argument) { //send socket lokalisasi
        while (1) {
                //sleep(1);
		usleep(500000);
                if (useAutoDirection) {
                        arahGoal();
                        if (uniV == 1) { //selatan
                                sendLokalisasi(walkX, walkY, walkA, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, Utara, 360-strategy.bearingValue, Ball_d, 57.29*headPan);
                        } else { //utara
                                sendLokalisasi(walkX, walkY, walkA, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, Selatan, 360-strategy.bearingValue, Ball_d, 57.29*headPan);
                        }
                } else {
                        if (strategy.Uniform == 1) { //selatan
                                sendLokalisasi(walkX, walkY, walkA, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, Utara, 360-strategy.bearingValue, Ball_d, 57.29*headPan);
                        } else { //utara
                                sendLokalisasi(walkX, walkY, walkA, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, Selatan, 360-strategy.bearingValue, Ball_d, 57.29*headPan);
                        }
                }
        }
}*/

/*double	siA, taA;
double	jarakKoorX, jarakKoorY, jarakKoorR;
void gridLokalisasi() {
	if (receler.RobotCoor_X > 0 && receler.RobotCoor_X < 900 && receler.RobotCoor_Y > 0 && receler.RobotCoor_Y < 900) {
		gridX = int(receler.RobotCoor_X/100) + 1; //ditambah 1 untuk pembulatan keatas
		if (gridX <= 1) { gridX = 1; }
		else if (gridX >= 9) { gridX = 9; }

		gridY = int(receler.RobotCoor_Y/100) + 1; //ditambah 1 untuk pembulatan keatas
		if (gridY <= 1) { gridY = 1; }
		else if (gridY >= 6) { gridY = 6; }

		Grid = gridY + (6 * (gridX-1));
		if (Grid <= 1) { Grid = 1; }
		else if (Grid >= 54) { Grid = 54; }

		jarakKoorX = 900 - receler.RobotCoor_X;
		jarakKoorY = 300 - receler.RobotCoor_Y;
		taA = atan(jarakKoorY/jarakKoorX) * (180/PI);
		//jarakKoorR = sqrt((jarakKoorX*jarakKoorX) + (jarakKoorY*jarakKoorY));
		//siA = asin(jarakKoorY/jarakKoorR) * (180/PI);
		//printf("  %f %f | sin = %f, tan = %f\n", jarakKoorX, jarakKoorY, siA, taA);
	} else {
		gridX = gridY = Grid = 0;
	}*/ //printf("\n  %d > %d | %d > %d | Grid = %d", receler.RobotCoor_X, gridX, receler.RobotCoor_Y, gridY, Grid);

	// baris pertama
	/*if (Grid == 54) { estimasiAngle = 90; }
	else if (Grid == 53) { estimasiAngle = 50; }
	else if (Grid == 50) { estimasiAngle = -50; }
	else if (Grid == 49) { estimasiAngle = -90; }
	// baris kedua
	else if (Grid == 48) { estimasiAngle = 50; }
	else if (Grid == 47) { estimasiAngle = 30; }
	else if (Grid == 44) { estimasiAngle = -30; }
	else if (Grid == 43) { estimasiAngle = -50; }
	// baris ketiga
	else if (Grid == 42) { estimasiAngle = 40; }
	else if (Grid == 41) { estimasiAngle = 20; }
	else if (Grid == 38) { estimasiAngle = -20; }
	else if (Grid == 37) { estimasiAngle = -40; }*/
	// jika pakai auto angle
	/*if (Grid > 36) { estimasiAngle = int(taA) * -1; } //test dulu
	// diluar dari tiga baris depan
	else {
		estimasiAngle = 0;
	}
}*/

//main
// Coordinations ===============================================================================
int	delayTrackGoal = 0;
bool	goalFound = false,
	breaked = false;
void myTurn() {
	//if (useFollowSearchGoal && useSearchGoal) {
	if (useFollowSearchGoal && followSearchAktif) {
		if (angle >= -80 && angle <= 80) { //angle dan jarak terpenuhi
			if (chotto > 20) {
				FollowGoal = true;
			} else {
				chotto++;
			}
		}

		if (FollowGoal) {
			Activated = false;
			if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, 232, robotStatus); }

			if (!searchGoalFinish) { //pertama
				if (!ballposTracked) { //pertama
					if (ballLost(20)) {
						if (waiting > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} waiting++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						if (headTilt > -1.4 && !useNearFollowSearchGoal) { //kelewatan
							delay = 0;
							tunggu = 0;

							prediksiGoalPan = 0;
							saveAngle = 0;

							goalFound = false;
							goalSearch = true;

							Walk(kejar, 0.0, 0.0);

							ballposTracked = true;
							searchGoalFinish = true;
						} else if (headTilt >= -1.7 && headPan >= -0.1 && headPan <= 0.1) {
							delay = 0;

							Walk(kejar, 0.0, 0.0);
							//followBall(0);

							if (delayTrackBall > 20) {
								ballposTracked = true;
							} else { delayTrackBall++; }
						} else { //masih jauh
							delayTrackBall = 0;

							prediksiGoalPan = 0;
							saveAngle = 0;

							goalFound = false;
							goalSearch = false;

							followBall(0);
						}
					}
				} else { //kedua
					if (!goalSearch) { //pertama
						tunggu = 0;
						if (delay > 5) {
							cekWaktu(6);
							if (ballTilt < -1.6) { //jarak jauh
								Walk(0.06, 0.0, 0.0);
							} else if (ballTilt < -1.5 && ballTilt >= -1.6) { //jarak sedang
								if (second < 4.5) { //sambil kejar
									Walk(0.05, 0.0, 0.0);
								} else { //space sebelum sampai ke bola (biar gk nabrak bola)
									Walk(0.0, 0.0, 0.0);
								}
							} else { //jarak dekat
								if (second < 3) { //sambil kejar //3.5
									Walk(0.05, 0.0, 0.0);
								} else { //space sebelum sampai ke bola (biar gk nabrak bola)
									Walk(0.0, 0.0, 0.0);
								}
							}

							if (goalLost(20)) { //pertama
								delayTrackGoal = 0;
								if (!timer) { //pertama
									if (second <= 2.0) { //search goal sambil kejar
										predictGoal(angle, -2.0); //tengah
									} else if (second > 4.0) {
										predictGoal(angle+30, -2.0); //kanan
									} else {
										predictGoal(angle-30, -2.0); //kiri
									}
								} else { //kedua
									prediksiGoalPan = 0;
									saveAngle = 0;

									if (ballTilt < -1.6) {
										loadBallLocation(0.2); //0.15
									} else if (ballTilt >= -1.5 && ballTilt >= -1.6) {
										loadBallLocation(0.4); //0.35
									} else {
										loadBallLocation(0.6);
									}

									goalFound = false;
									goalSearch = true;
								}
							} else { //kedua
								if (timer || delayTrackGoal > 50) { //kedua
									if (((saveAngle - prediksiGoalPan) > 90) || ((saveAngle - prediksiGoalPan) < -90)) {
										prediksiGoalPan = 0;
										saveAngle = 0;
									}

									if (ballTilt < -1.6) {
										loadBallLocation(0.2); //0.15
									} else if (ballTilt >= -1.5 && ballTilt >= -1.6) {
										loadBallLocation(0.4); //0.35
									} else {
										loadBallLocation(0.6);
									}

									goalFound = true;
									goalSearch = true;
								} else { //pertama
									trackGoal();
									prediksiArahGoal();
									saveSudutImu();
									if(useUpdateCoordinate) { updateCoordinatFromVision(); }		
									delayTrackGoal++;
								}
							}
						} else {
							trackBall();
							Walk(kejar, 0.0, 0.0);			
							setWaktu();
							delayTrackGoal = 0;						
							delay++;
						}
					} else { //kedua
						if (tunggu < 5) { //20 //pertama				
							trackBall();
							delay = 0;
							waiting = 0;
							robotDirection = false;
							setWaktu();
							tunggu++;
						} else { //kedua
							if (ballLost(20)) {
								if (waiting > 40) {
									resetCase1();
									stateCondition = 1;
								} else {
									tiltSearchBall(0.0);
								} waiting++;
								Walk(0.0, 0.0, 0.0);
							} else {
								trackBall();
								waiting = 0;

								if (goalFound == true) {
									if (robotDirection && headPan >= -0.4 && headPan <= 0.4) { //kedua
										//if (headTilt >= (cSekarang - 0.2)) {
											Walk(0.0, 0.0, 0.0);
											searchGoalFinish = true;
										//} else {
										//	if (headTilt >= -1.0) {
										//		ballPositioning(0.0, cSekarang, 0.12);
										//	} else {
										//		followBall(0);
										//	}
										//}
									} else { //pertama
										if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) { //kedua
											if (delay > 5) {
												cekWaktu(20);
												if (timer) {
													robotDirection = true;
												} else {
													if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30)) {
														if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
														else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
														if (errorGoal >= 20) { errorGoal = 20; }
													} else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30)) {
														if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
														else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
														if (errorGoal >= 20) { errorGoal = 20; }
													} else {
														errorGoal = 0;
														//if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
														//else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
														//if (errorGoal >= 20) { errorGoal = 20; }
													}

													if (useSideKick) {
														/*if (saveAngle >= 45) { //50
															if (((saveAngle - prediksiGoalPan) + errorGoal) <= 45) { //30
																modeKick = 4; //tendangSamping
																Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
															} else {
																modeKick = tendangJauh;
																Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
																lastDirection = angle;
															}
														} else if (saveAngle <= -45) { //-50
															if (((saveAngle - prediksiGoalPan) + errorGoal) >= -45) { //-30
																modeKick = 3; //tendangSamping
																Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
															} else {
																modeKick = tendangJauh;
																Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
																lastDirection = angle;
															}
														} else {
															if (((saveAngle - prediksiGoalPan) + errorGoal) >= 45) { //50
																modeKick = 3; //tendangSamping
																Imu((saveAngle - prediksiGoalPan) + errorGoal - 90, cSekarang);
															} else if (((saveAngle - prediksiGoalPan) + errorGoal) <= -45) { //-50
																modeKick = 4; //tendangSamping
																Imu((saveAngle - prediksiGoalPan) + errorGoal + 90, cSekarang);
															} else {
																modeKick = tendangJauh;
																Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
																lastDirection = angle;
															}
														}*/

														if (prediksiGoalPan >= 45) { //kiri
															modeKick = 4; //tendangSamping
															Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														} else if (prediksiGoalPan <= -45) { //kanan
															modeKick = 3; //tendangSamping
															Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														} else {
															modeKick = tendangJauh;
															Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
															lastDirection = angle;
														}
													} else {
														modeKick = tendangJauh;
														Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														lastDirection = angle;
													}
												}
											} else {
												setWaktu();
												robotDirection = false;
												delay++;
											}
										} else { //pertama
											delay = 0;
											followBall(0);
										}
									}
								} else {
									searchGoalFinish = true;
								}
							}
						}
					}
				}
			} else { //kedua (next case)
				if (ballLost(20)) {
					if (tunggu > 40) {
						resetCase1();
						stateCondition = 1;
					} else {
						tiltSearchBall(0.0);
					} tunggu++;
					Walk(0.0, 0.0, 0.0);
				} else {
					trackBall();
					tunggu = 0;

					if (headTilt >= cAktif) {
						//if (goalFound || (gridX == 9 && useLocalization == true)) {
						if (goalFound) {
							resetCase5();
							stateCondition = 5;
						} else {
							resetCase7();
							stateCondition = 7;
						}
					} else {
						followBall(0);
					}
				}
			}
		} else { //jika gawang dibelakang (next case)
			if (ballLost(20)) {
				if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus); }
				if (tunggu > 40) {
					resetCase1();
					stateCondition = 1;
				} else {
					tiltSearchBall(0.0);
				} tunggu++;
				Walk(0.0, 0.0, 0.0);
			} else {
				trackBall();
				if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, 232, robotStatus); }
				tunggu = 0;

				if (headTilt >= cAktif) {
					resetCase7();
					stateCondition = 7;
				} else {
					followBall(0);
				}
			}
		}
	} else { //without follow search goal (next case)
		if (ballLost(20)) {
			if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus); }
			if (tunggu > 40) {
				resetCase1();
				stateCondition = 1;
			} else {
				tiltSearchBall(0.0);
			} tunggu++;
			Walk(0.0, 0.0, 0.0);
		} else {
			trackBall();
			if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, 232, robotStatus); }
			tunggu = 0;

			if (headTilt >= cAktif) {
				resetCase7();
				stateCondition = 7;
			} else {
				followBall(0);
			}
		}
	}
}

void waitingTurn() {
	if (ballLost(20)) {
		if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus); }

		if (tunggu > 40) {
			resetCase1();
			stateCondition = 1;
		} else {
			tiltSearchBall(0.0);
			Walk(0.0, 0.0, 0.0);
		} tunggu++;
	} else {
		trackBall();
		if (useCoordination) { koordinasi.sendRobotCoordinationData(robotID, 1, semeh, stateCondition, robotStatus); }

		setWaktu();
		delay = 0;
		tunggu = 0;
		robotDirection = false;

		//if (headTilt > -1.3 && headPan >= -0.4 && headPan <= 0.4) {
		//	Walk(-0.03, 0.0, 0.0);
		//} else if (headTilt >= -1.5 && headTilt <= -1.3 && headPan >= -0.4 && headPan <= 0.4) {
		if (headTilt > -1.5 && headPan >= -0.4 && headPan <= 0.4) {
			//if (angle >= 0) { Imu(90, -1.4); }
			//else { Imu(-90, -1.4); }
			if (angle >= 0) { //Imu 90
				if (angle > 80 && angle < 100) {
					if (headTilt > -1.3) {
						bodyXImu = -0.03; alfaImu = 0.0;
					} else {
						bodyXImu = alfaImu = 0.0;
					}
					bodyYImu = errorCPosPan * 0.06;
				} else if ((angle <= 90) && (angle >= -90)) {
					rotateDirec(-1, -1.4); //printf("  rotate ke kanan\n\n");
				} else {
					rotateDirec(1, -1.4); //printf("  rotate ke kiri\n\n");
				} Walk(bodyXImu, bodyYImu, alfaImu);
			} else { //Imu -90
				if (angle < -80 && angle > -100) {
					if (headTilt > -1.3) {
						bodyXImu = -0.03; alfaImu = 0.0;
					} else {
						bodyXImu = alfaImu = 0.0;
					}
					bodyYImu = errorCPosPan * 0.06;
				} else if ((angle <= 90) && (angle >= -90)) {
					rotateDirec(1, -1.4); //printf("  rotate ke kiri\n\n");
				} else {
					rotateDirec(-1, -1.4); //printf("  rotate ke kanan\n\n");
				} Walk(bodyXImu, bodyYImu, alfaImu);
			}
		} else {
			followBall(0);
		}
	}
}

//main
void yourTurn() {
	if (robotID == 1) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 2) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 3) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 4) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 5) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	}
}

int	gommen = 0, maxRef = 800,
	countR1 = 0, countR2 = 0, countR3 = 0, countR4 = 0, countR5 = 0,
	lastR1StateBall = 0, lastR2StateBall = 0, lastR3StateBall = 0, lastR4StateBall = 0, lastR5StateBall = 0;
void refreshComm() {
	if (countR1 >= maxRef || countR2 >= maxRef || countR3 >= maxRef || countR4 >= maxRef || countR5 >= maxRef) {
		//printf("\n.....................................................................refresh\n");
		koordinasi.refresh = true;
		if (robotID != 1) { koordinasi.robot1Id = koordinasi.robot1FBall = koordinasi.robot1State = koordinasi.robot1Status = 0; koordinasi.robot1DBall = 232; }
		if (robotID != 2) { koordinasi.robot2Id = koordinasi.robot2FBall = koordinasi.robot2State = koordinasi.robot2Status = 0; koordinasi.robot2DBall = 232; }
		if (robotID != 3) { koordinasi.robot3Id = koordinasi.robot3FBall = koordinasi.robot3State = koordinasi.robot3Status = 0; koordinasi.robot3DBall = 232; }
		if (robotID != 4) { koordinasi.robot4Id = koordinasi.robot4FBall = koordinasi.robot4State = koordinasi.robot4Status = 0; koordinasi.robot4DBall = 232; }
		if (robotID != 5) { koordinasi.robot5Id = koordinasi.robot5FBall = koordinasi.robot5State = koordinasi.robot5Status = 0; koordinasi.robot5DBall = 232; }
		countR1 = countR2 = countR3 = countR4 = countR5 = 0;
	} else {
		koordinasi.refresh = false;
		if (robotID != 1) {
			if (koordinasi.robot1State != 1 && koordinasi.robot1State != 150 && koordinasi.robot1State != 50 && koordinasi.robot1State != 90 && koordinasi.robot1State != 100 && lastR1StateBall == koordinasi.robot1State) { countR1++; }
			else { countR1 = 0; lastR1StateBall = koordinasi.robot1State; }
		}
		if (robotID != 2) {
			if (koordinasi.robot2State != 1 && koordinasi.robot2State != 150 && koordinasi.robot2State != 50 && koordinasi.robot2State != 90 && koordinasi.robot2State != 100 && lastR2StateBall == koordinasi.robot2State) { countR2++; }
			else { countR2 = 0; lastR2StateBall = koordinasi.robot2State; }
		}
		if (robotID != 3) {
			if (koordinasi.robot3State != 1 && koordinasi.robot3State != 150 && koordinasi.robot3State != 50 && koordinasi.robot3State != 90 && koordinasi.robot3State != 100 && lastR3StateBall == koordinasi.robot3State) { countR3++; }
			else { countR3 = 0; lastR3StateBall = koordinasi.robot3State; }
		}
		if (robotID != 4) {
			if (koordinasi.robot4State != 1 && koordinasi.robot4State != 150 && koordinasi.robot4State != 50 && koordinasi.robot4State != 90 && koordinasi.robot4State != 100 && lastR4StateBall == koordinasi.robot4State) { countR4++; }
			else { countR4 = 0; lastR4StateBall = koordinasi.robot4State; }
		}
		if (robotID != 5) {
			if (koordinasi.robot5State != 1 && koordinasi.robot5State != 150 && koordinasi.robot5State != 50 && koordinasi.robot5State != 90 && koordinasi.robot5State != 100 && lastR5StateBall == koordinasi.robot5State) { countR5++; }
			else { countR5 = 0; lastR5StateBall = koordinasi.robot5State; }
		}
	} //printf("\n  %d,%d,%d,%d,%d\n", countR1, countR2, countR3, countR4, countR5);
}

void backToCoordinations() {
	if (robotID == 1) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 2) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 3) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 4) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 5) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	}
}

// normal search Ball ========================================================================
bool	posRotasi = false;
int	rotasi = 0;
void rotateSearchBall(int rotate) {
	if (rotate >= 0) {
		rotasi = rotate - 180;
		//goalDirec(rotasi); //Dipakai jika menggunakan magnetometer
		sudut();

		setPoint1 =  20 + rotasi; //20
		setPoint2 = -20 + rotasi; //20

		if (angle > setPoint2 && angle < setPoint1 ) {
			posRotasi = true;
		} else {
			//jalanDirection(0.0, 0.0, rotasi);
			Walk(0.0, 0.0, 0.27);
		}
	} else {
		rotasi = rotate + 180;
		//goalDirec(rotasi); //Dipakai jika menggunakan magnetometer
		sudut();

		setPoint1 =  20 + rotasi; //20
		setPoint2 = -20 + rotasi; //20

		if (angle > setPoint2 && angle < setPoint1 ) {
			posRotasi = true;
		} else {
			//jalanDirection(0.0, 0.0, rotasi);
			Walk(0.0, 0.0, 0.27);
		}
	}
}

bool	firstRotate  = false,
	secondRotate = false,
	thirdRotate = false,
	fourthRotate = false,

	firstWalk = false,
	secondWalk = false;
void normalSearchBall() {
	//mode1
	/*if (!firstRotate) { //ini step yang pertama kali dilakukan saat masuk case 0
		if (searchKe >= 5) {
			Walk(0.0, 0.0, 0.0); //X, Y, W
			firstRotate  = true;
			searchKe = 0;
		} else if (searchKe >= 2 && searchKe < 5) { //printf("  2.........\n\n");
			SearchBall(2);
			//tiltSearchBall(0.0);
			Walk(0.0, 0.0, 0.15);
		} else { //printf("  1.........\n\n");
			SearchBall(2);
			Walk(0.0, 0.0, 0.0); //X, Y, W
		}
	} else { //setelah rotate pertama tidak dapat, maka cari sambil jalan(dengan kompas)
		if (!secondRotate) { //yang pertama kali dilakukan
			if (searchKe >= 3) { //printf("  4.........\n\n");
				SearchBall(2);
				//tiltSearchBall(0.0);
				Walk(0.0, 0.0, 0.12);

				if (searchKe >= 5) {
					searchKe = 0;
					secondRotate  = true;
				}
			} else { //printf("  3.........\n\n");
				SearchBall(2);
				Walk(kejar, 0.0, 0.0);
			}
		} else { //jalan dengan arah sebaliknya
			if (searchKe >= 6) { //printf("  6.........\n\n");
				SearchBall(2);
				if (searchKe >= 8) {
					if (searchKe > 12) {
						jalanDirection(kejar, 0.0, 0);
					} else {
						//searchKe = 0;
						//firstRotate = secondRotate = false;
						Walk(kejar, 0.0, 0.0);
					}
				} else {
					//tiltSearchBall(0.0);
					Walk(0.0, 0.0, 0.12);
				}
			} else { //printf("  5.........\n\n");
				SearchBall(2);
				Walk(kejar, 0.0, 0.0);
			}
		}
	}*/

	//mode 2
	if (!firstRotate) {
		if (matte > 5) {
			if (searchKe == 1) {
				if (posRotasi) {
					//Walk(0.0, 0.0, 0.0);
					jalanDirection(0.0, 0.0, rotasi);
					matte = 0;
					firstRotate = true;
				} else {
					//headMove(0.0, -1.6);
					threeSearchBall();
					rotateSearchBall(saveAngle);
				}
			} else {
				saveSudutImu();
				threeSearchBall();
				Walk(0.0, 0.0, 0.0);
			}
		} else {
			posRotasi = false;
			sabar = 0;
			searchKe = 0;
			matte++;
		}
	} else {
		if (!secondRotate) {
			if (matte > 5) {
				if (searchKe == 1) {
					if (posRotasi) {
						//Walk(0.0, 0.0, 0.0);
						jalanDirection(0.0, 0.0, rotasi);
						matte = 0;
						secondRotate = true;
					} else {
						//headMove(0.0, -1.6);
						threeSearchBall();
						rotateSearchBall(saveAngle);
					}
				} else {
					saveSudutImu();
					threeSearchBall();
					//Walk(0.0, 0.0, 0.0);
					jalanDirection(0.0, 0.0, saveAngle);
				}
			} else {
				posRotasi = false;
				sabar = 0;
				searchKe = 0;
				matte++;
			}
		} else {
			if (!firstWalk) {
				if (matte > 5) {
					if (searchKe == 2) {
						//posRotasi = false;
						saveSudutImu();
						//Walk(0.0, 0.0, 0.0);
						jalanDirection(0.0, 0.0, saveAngle);
						matte = 0;
						firstWalk = true;
					} else {
						threeSearchBall();
						//Walk(kejar, 0.0, 0.0);
						jalanDirection(kejar, 0.0, rotasi);
					}
				} else {
					posRotasi = false;
					sabar = 0;
					searchKe = 0;
					matte++;
				}
			} else {
				if (!thirdRotate) {
					if (matte > 5) {
						if (posRotasi) {
							//Walk(0.0, 0.0, 0.0);
							jalanDirection(0.0, 0.0, rotasi);
							//sabar = 0;
							//searchKe = 0;
							matte = 0;
							thirdRotate = true;
						} else {
							threeSearchBall();
							rotateSearchBall(saveAngle);
						}
					} else {
						posRotasi = false;
						sabar = 0;
						searchKe = 0;
						matte++;
					}
				} else {
					if (!secondWalk) {
						if (matte > 5) {
							if (searchKe == 4) {
								//posRotasi = false;
								saveSudutImu();
								//Walk(0.0, 0.0, 0.0);
								jalanDirection(0.0, 0.0, rotasi);
								matte = 0;
								secondWalk = true;
							} else {
								threeSearchBall();
								//Walk(kejar, 0.0, 0.0);
								jalanDirection(kejar, 0.0, rotasi);
							}
						} else {
							posRotasi = false;
							sabar = 0;
							searchKe = 0;
							matte++;
						}
					} else {
						if (!fourthRotate) {
							if (matte > 5) {
								if (posRotasi) {
									//Walk(0.0, 0.0, 0.0);
									jalanDirection(0.0, 0.0, rotasi);
									//sabar = 0;
									//searchKe = 0;
									matte = 0;
									fourthRotate = true;
								} else {
									threeSearchBall();
									rotateSearchBall(saveAngle);
								}
							} else {
								posRotasi = false;
								sabar = 0;
								searchKe = 0;
								matte++;
							}
						} else {
							if (matte > 5) {
								threeSearchBall();
								//Walk(kejar, 0.0, 0.0);
								jalanDirection(kejar, 0.0, rotasi);
							} else {
								posRotasi = false;
								sabar = 0;
								searchKe = 0;
								matte++;
							}
						}
					}
				}
			}
		}
	}
	//mode 3 pake jalan direction masih bug, robot diam tiba search >=2
	/*
	if (!firstRotate) {
		if (searchKe == 1) {
			if (posRotasi) {
				Walk(0.0, 0.0, 0.0);
				sabar = 0;
				searchKe = 0;
				firstRotate = true;
			} else {
				//headMove(0.0, -1.6);
				threeSearchBall();
				rotateSearchBall(saveAngle);
			}
		} else {
			posRotasi = false;
			saveSudutImu();
			threeSearchBall();
			Walk(0.0, 0.0, 0.0);
		}
	} else {
		if (!secondRotate) {
			if (searchKe == 1) {
				if (posRotasi) {
					//Walk(0.0, 0.0, 0.0);
					jalanDirection(0.0, 0.0, saveAngle);
					sabar = 0;
					searchKe = 0;
					secondRotate = true;
				} else {
					//headMove(0.0, -1.6);
					threeSearchBall();
					rotateSearchBall(saveAngle);
				}
			} else {
				posRotasi = false;
				saveSudutImu();
				threeSearchBall();
				//Walk(0.0, 0.0, 0.0);
				jalanDirection(0.0, 0.0, saveAngle);
			}
		} else {
			if (!firstWalk) {
				if (searchKe == 2) {
					posRotasi = false;
					saveSudutImu();
					//Walk(0.0, 0.0, 0.0);
					jalanDirection(0.0, 0.0, saveAngle);
					firstWalk = true;
				} else {
					threeSearchBall();
					//Walk(kejar, 0.0, 0.0);
					jalanDirection(kejar, 0.0, rotasi);
				}
			} else {
				if (!thirdRotate) {
					if (posRotasi) {
						//Walk(0.0, 0.0, 0.0);
						jalanDirection(0.0, 0.0, saveAngle);
						sabar = 0;
						searchKe = 0;
						thirdRotate = true;
					} else {
						//headMove(0.0, -1.6);
						threeSearchBall();
						rotateSearchBall(saveAngle);
					}
				} else {
					if (!secondWalk) {
						if (searchKe == 4) {
							posRotasi = false;
							saveSudutImu();
							//Walk(0.0, 0.0, 0.0);
							jalanDirection(0.0, 0.0, rotasi);
							secondWalk = true;
						} else {
							threeSearchBall();
							//Walk(kejar, 0.0, 0.0);
							jalanDirection(kejar, 0.0, rotasi);
						}
					} else {
						if (!fourthRotate) {
							if (posRotasi) {
								//Walk(0.0, 0.0, 0.0);
								jalanDirection(0.0, 0.0, rotasi);
								sabar = 0;
								searchKe = 0;
								fourthRotate = true;
							} else {
								//headMove(0.0, -1.6);
								threeSearchBall();
								rotateSearchBall(saveAngle);
							}
						} else {
							threeSearchBall();
							//Walk(kejar, 0.0, 0.0);
							jalanDirection(kejar, 0.0, rotasi);
						}
					}
				}
			}
		}
	}
*/
//main
	/*if (reset > 5) {
		cekWaktu(0);
		if (second <= 7) { //7
			saveSudutImu();
			searchBallRectang(-1.4, -1.6, -0.8, 1.6);
			Walk(0.0, 0.0, 0.0);
		} else if (second > 7 && second <= 24) { //17
			//searchRectangle(-1.4, -1.6, -0.8, 1.6);
			tiltSearchBall(0.0);
			Walk(0.0, 0.0, 0.15); //0.15
		} else if (second > 24 && second <= 44) { //20
			searchBallRectang(-1.4, -1.6, -0.8, 1.6);
			if (saveAngle > -90 && saveAngle < 90) {
				jalanDirection(jalan, 0.0, 0);
			} else {
				jalanDirection(jalan, 0.0, 180);
			}
		} else if (second > 44 && second <= 84) { //40
			searchBallRectang(-1.4, -1.6, -0.8, 1.6);
			if (saveAngle > -90 && saveAngle < 90) {
				jalanDirection(jalan, 0.0, 180);
			} else {
				jalanDirection(jalan, 0.0, 0);
			}
		} else if (second > 84 && second <= 104) { //20
			searchBallRectang(-1.4, -1.6, -0.8, 1.6);
			if (saveAngle > -90 && saveAngle < 90) {
				jalanDirection(jalan, 0.0, 0);
			} else {
				jalanDirection(jalan, 0.0, 180);
			}
		} else {
			reset = 0;
		}
	} else {
		setWaktu();
		searchBallRectang(-1.4, -1.6, -0.8, 1.6);
		Walk(0.0, 0.0, 0.0);
		reset++;
	}*/
}

// Reset Variabel ============================================================================
void resetCaseAwal() { //strategi awal
	kondisiBola		=
	elapsedTime		=
	second			=
	elapsedTime2		=
	second2			=
	reset			=
	cntOke1			=
	cntOke2			=
	cntUlang		=
	waitingBall 		=
	delay 			=
	count 			=
	tunggu 			=
	countPickUp 		=
	arahRotate		=
	searchTime		=
	rotateKick 		=
	Waktu			=
	Timing			= 0;

	modeKick		= tendangJauh;

	ulang			=
	followSearchAktif	= true;

	dapat			=
	GoalTracked		=
	timer			=
	timer2			=
	oke			=
	tracked			=
	posRotate		=
	robotDirection		=
	tendang			= false;
}

void resetCase1() { //search ball
	panRate1	= -0.06;
	panRate2	= -0.08;
	panRate3	= -0.1;
	//i		= 1;

	reset		=
	tunda		=
	saveAngle	=
	searchKe	=	//counting berapa kali search
	delayWaitBall	=	//delay memastikan bola
	rotasi		=
	sabar		=
	matte		= 0;
	//tiltKe		= 0;

	posRotasi	=
	firstRotate	=
	secondRotate	=
	thirdRotate	=
	fourthRotate	=
	firstWalk	=
	secondWalk	=
	tracked		= false;
}

void resetCase2() { //cek koordinasi, follow, n cari gawang
	searchKe		=
	tunggu			=
	delay			=
	delayTrackBall		=
	elapsedTime		=
	second			=
	reset			=
	waiting			=
	errorGoal		=
	goalPan			=
	saveAngle		=
	setPointAngle		=
	prediksiGoalPan		=
	chotto			= 0;

	modeKick		= tendangJauh;

	Activated		= true;

	exeCutor		=
	FollowGoal		=
	searchGoalFinish	=
	timer			=
	ballPos			=
	goalSearch		=
	ballposTracked		=
	goalFound		=
	breaked			=
	robotDirection		= false;
}

void resetCase7() { //compass
	elapsedTime	=
	second		=
	reset		=
	delay		=
	delayTrackGoal	=
	tunggu		= 0;

	modeKick	= tendangJauh;

	timer		=
	robotDirection	= false;
}

void resetCase3() { //dribble
	searchKe	=
	tunggu		=
	delay 		=
	countDribble	=
	setPointAngle	= 0;

	robotDirection	= false;
}

void resetCase8() { //robot must to check direction again after dribble
	searchKe	=
	tunggu		=
	delay		=
	setPointAngle	= 0;

	robotDirection	= false;
}

void resetCase4() { //search goal
	searchKe		=
	tunggu			=
	Rotate			=
	delay			=
	waiting			=
	elapsedTime		=
	second			=
	reset			=
	count			=
	bodyP_ControllerG	=
	bodyTrueG		=
	delayTrueG		=
	errorGoal		=
	goalPan			=
	saveAngle		=
	prediksiGoalPan		=
	thisOffset		=
	headOffset		= 0;

	modeKick		= tendangJauh;

	searchGoalFinish	=
	timer			=
	ballPos			=
	goalSearch		=
	rotateGoal		=
	ballposTracked		=
	errorGoal		=
	robotDirection		= false;
}

void resetCase5() { //kick
	searchKe	=
	delay		=
	tunggu		= 0;

	kanan		=
	kiri		=
	ballPos 	=
	tendang 	= false;
}

void resetCase6() { //search after kick
	searchKe	=
	tunda		=
	tunggu		=
	elapsedTime	=
	second		=
	delay		=
	confirmsBall	=
	sumTilt		=
	sumPan		=
	waitTracking	=
	countTilt	= 0;

	neckX		=
	Move		= true;

	searchRectangle =
	tendangLagi	=
	timer		= false;
}

void resetOdometry(){
	robotPos_X		=
	robotPos_Y		=
	odometry.ArchSin	=
	odometry.csmKiri	=
	odometry.csmKanan	=
	odometry.walkTot	=
	odometry.deltaPos_X	=
	varCount		=
	odometry.deltaPos_Y	=
	odometry.walkTotMun	= 0;
}

void refreshMoveLokalisasi(){
	cL = 	//count lock for atribut target
	cD =	//count peralihan dari rotate ke jalan
	cR =	//count peralihan dari break ke rotate
	cZ = cV = cN = cY =
	cB = 0;	//count untuk break ketika sudah didalam setpoint target
	cX = false,
	donePosition = false;
}

void resetKoordinasiRobotBalik(){
	balikTengah = false;
	balikSampingKanan = false;
	balikSampingKiri = false;
}

void refreshRecelerLokalisasi(){
	receler.RobotCoor_X	=
	receler.RobotCoor_Y	=
	receler.BallCoor_X	=
	receler.BallCoor_Y	= 0;
}

void resetAllLokalisasiVariable() {
	lockLeftBack	=
	lockRightBack	=
	kembaliMasuk	=
	lockMidBack 	= 
	awalMasuk	= false;

	sebentar	= 0;

	//resetOdometry();
	//refreshRecelerLokalisasi();
	refreshMoveLokalisasi();
}

void resetAllVariable() { //setelah tendang
	gridX		=
	gridY		=
	Grid		=
	estimasiAngle	=
	countInitPosPen =  0;

	resetCaseAwal();
	resetCase1();
	resetCase2();
	resetCase3();
	resetCase4();
	resetCase5();
	resetCase6();
	resetCase7();
	resetCase8();

	kembaliMasuk = false,
	awalMasuk = false,
	lockLeftBack = false,
	lockMidBack 	= false,
	lockRightBack = false;

	resetAllLokalisasiVariable();
}

// Load Parameter configure.ini ==============================================================
double value = 0;
void loadConfig(){
	//geti -> tipe data INT
	//getd -> tipe data double
	//getf -> tipe data float
	//getl -> tipe data long

	if((value = ini -> geti("config", "robotNumber", INVALID_VALUE)) != INVALID_VALUE)  	robotNumber = value;
	if((value = ini -> geti("config", "frame_X", INVALID_VALUE)) != INVALID_VALUE)  	frame_X = value;
	if((value = ini -> geti("config", "frame_Y", INVALID_VALUE)) != INVALID_VALUE)  	frame_Y = value;
	if((value = ini -> geti("config", "Utara", INVALID_VALUE)) != INVALID_VALUE)   		Utara = value;
	if((value = ini -> geti("config", "Timur", INVALID_VALUE)) != INVALID_VALUE)   		Timur = value;
	if((value = ini -> geti("config", "Selatan", INVALID_VALUE)) != INVALID_VALUE)   	Selatan = value;
	if((value = ini -> geti("config", "Barat", INVALID_VALUE)) != INVALID_VALUE)   		Barat = value;
	if((value = ini -> geti("config", "tendangJauh", INVALID_VALUE)) != INVALID_VALUE)   	tendangJauh = value;
	if((value = ini -> geti("config", "tendangDekat", INVALID_VALUE)) != INVALID_VALUE)   	tendangDekat = value;
	if((value = ini -> geti("config", "tendangSamping", INVALID_VALUE)) != INVALID_VALUE)   tendangSamping = value;
	if((value = ini -> geti("config", "sudutTengah", INVALID_VALUE)) != INVALID_VALUE)   	sudutTengah = value;
	if((value = ini -> geti("config", "sudutKanan", INVALID_VALUE)) != INVALID_VALUE)   	sudutKanan = value;
	if((value = ini -> geti("config", "sudutKiri", INVALID_VALUE)) != INVALID_VALUE)   	sudutKiri = value;

	if((value = ini -> getd("config", "ball_panKP", INVALID_VALUE)) != INVALID_VALUE)	ball_panKP = value;
	if((value = ini -> getd("config", "ball_panKD", INVALID_VALUE)) != INVALID_VALUE)   	ball_panKD = value;
	if((value = ini -> getd("config", "ball_tiltKP", INVALID_VALUE)) != INVALID_VALUE)	ball_tiltKP = value;
	if((value = ini -> getd("config", "ball_tiltKD", INVALID_VALUE)) != INVALID_VALUE)   	ball_tiltKD = value;

	if((value = ini -> getd("config", "goal_panKP", INVALID_VALUE)) != INVALID_VALUE)	goal_panKP = value;
	if((value = ini -> getd("config", "goal_panKD", INVALID_VALUE)) != INVALID_VALUE)   	goal_panKD = value;
	if((value = ini -> getd("config", "goal_tiltKP", INVALID_VALUE)) != INVALID_VALUE)	goal_tiltKP = value;
	if((value = ini -> getd("config", "goal_tiltKD", INVALID_VALUE)) != INVALID_VALUE)   	goal_tiltKD = value;

	if((value = ini -> getd("config", "ballPositioningSpeed", INVALID_VALUE)) != INVALID_VALUE) ballPositioningSpeed = value;
	if((value = ini -> getd("config", "pPanTendang", INVALID_VALUE)) != INVALID_VALUE)   	pPanTendang = value;
	if((value = ini -> getd("config", "pTiltTendang", INVALID_VALUE)) != INVALID_VALUE)   	pTiltTendang = value;
	if((value = ini -> getd("config", "pPanOper", INVALID_VALUE)) != INVALID_VALUE)   	pPanOper = value;
	if((value = ini -> getd("config", "pTiltOper", INVALID_VALUE)) != INVALID_VALUE)   	pTiltOper = value;
	if((value = ini -> getd("config", "cSekarang", INVALID_VALUE)) != INVALID_VALUE)   	cSekarang = value;
	if((value = ini -> getd("config", "cAktif", INVALID_VALUE)) != INVALID_VALUE)   	cAktif = value;
	if((value = ini -> getd("config", "gAktif", INVALID_VALUE)) != INVALID_VALUE)   	gAktif = value;
	if((value = ini -> getd("config", "posTiltLocal", INVALID_VALUE)) != INVALID_VALUE)   	posTiltLocal = value;
	if((value = ini -> getd("config", "posTiltGoal", INVALID_VALUE)) != INVALID_VALUE)   	posTiltGoal = value;
	if((value = ini -> getd("config", "batasTilt", INVALID_VALUE)) != INVALID_VALUE)   	batasTilt = value;
	if((value = ini -> getd("config", "erorrXwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrXwalk = value;
	if((value = ini -> getd("config", "erorrYwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrYwalk = value;
	if((value = ini -> getd("config", "erorrAwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrAwalk = value;
	if((value = ini -> getd("config", "jalan", INVALID_VALUE)) != INVALID_VALUE)   		jalan = value;
	if((value = ini -> getd("config", "lari", INVALID_VALUE)) != INVALID_VALUE)   		lari = value;
	if((value = ini -> getd("config", "kejar", INVALID_VALUE)) != INVALID_VALUE)   		kejar = value;
	if((value = ini -> getd("config", "kejarMid", INVALID_VALUE)) != INVALID_VALUE)   	kejarMid = value;
	if((value = ini -> getd("config", "kejarMax", INVALID_VALUE)) != INVALID_VALUE)   	kejarMax = value;
	if((value = ini -> getd("config", "rotateGoal_x", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_x = value;
	if((value = ini -> getd("config", "rotateGoal_y", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_y = value;
	if((value = ini -> getd("config", "rotateGoal_a", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_a = value;
	if((value = ini -> getd("config", "myAccrX", INVALID_VALUE)) != INVALID_VALUE)   	myAccrX = value;
	if((value = ini -> getd("config", "myAccrY", INVALID_VALUE)) != INVALID_VALUE)   	myAccrY = value;
	if((value = ini -> getd("config", "tinggiRobot", INVALID_VALUE)) != INVALID_VALUE)   	tinggiRobot = value;
	if((value = ini -> getd("config", "outputSudutY1", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutY1 = value;
	if((value = ini -> getd("config", "inputSudutY1", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutY1 = value;
	if((value = ini -> getd("config", "outputSudutY2", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutY2 = value;
	if((value = ini -> getd("config", "inputSudutY2", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutY2 = value;
	if((value = ini -> getd("config", "outputSudutX1", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutX1 = value;
	if((value = ini -> getd("config", "inputSudutX1", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutX1 = value;
	if((value = ini -> getd("config", "outputSudutX2", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutX2 = value;
	if((value = ini -> getd("config", "inputSudutX2", INVALID_VALUE)) != INVALID_VALUE)  	inputSudutX2 = value;
}

void semahau(int pening) {
//	if (pening == 1) {
//	} else if (pening == 2) {
//	} else if (pening == 3) {
//	} else if (pening == 4) {
//	} else if (pening == 5) {
//	} else { pening = 1; }
}

void display() {
	//printf ("  detik = %.2f, weight = %d, step = %d, sumStep = %d, state = %d,", second, strategy.step, step, sumStep, stateCondition);
	//printf ("\n  detik = %.2f,\n", second2);
	printf ("\n  signIn = %d,\n", signIn);
	//printf ("\n  robotDirection = %d,", robotDirection);
	//printf ("  useImu = %d,\n", useImu);
	printf ("\n  RobotStatus = %d,\n", robotStatus);
	printf ("\n  zeroState = %d,\n", zeroState);
	//printf ("\n  pickUp = %d,\n", pickUp);
	//---------------------------------------------------------
	printf("\n  Ball_X = %d,  Ball_Y = %d, Ball_D = %d, \n", receler.Ball_X, receler.Ball_Y, receler.Ball_D);
	printf("  Goal_X = %d,  Goal_Y = %d, Goal_C = %d, Goal_LD = %d, Goal_RD = %d, \n", receler.Goal_X, receler.Goal_Y, receler.Goal_C, receler.Goal_LD, receler.Goal_RD);
	//printf("  initialPos_X = %2.lf, \n", initialPos_X);
	//printf("  initialPos_Y = %2.lf, \n", initialPos_Y);
	//---------------------------------------------------------
	printf("\n  stateGameController = %d,", stateGameController);
	//printf("  lastStateGameController = %d,", lastStateGameController);
	printf("  stateCondition = %d,", stateCondition);
	//printf("  stateChange = %d,\n", stateChange);
	printf("  Strategi = %d, \n", Strategi);
	//---------------------------------------------------------
	//printf("\n  gridX = %d, gridY = %d, Grid = %d,", gridX, gridY, Grid);
	//printf("\n  Grid = %d, estimasiAngle = %d,", Grid, estimasiAngle);
	//---------------------------------------------------------
	//printf("\n  kickOff = %d,", kickOff);
	//printf("  lastKickOff = %d,\n", lastKickOff);
	//---------------------------------------------------------
	//printf("\n  secsRemaining = %d,", secRemaining);
	//printf("  lastSecsRemaining = %d,\n", lastSecRemaining);
	//---------------------------------------------------------
	//getSensor();
	//printf("  accr(%.1lf, %.1lf, %.1lf),", accrX, accrY, accrZ);
	//printf("  gyro(%.lf, %.lf, %.lf),", gyroX, gyroY, gyroZ);
	//printf("  angle(%.lf, %.lf, %.lf),", angleX, angleY, angleZ);
	//---------------------------------------------------------
	//sudut();
	//printf("\n  Bearing = %d,", strategy.bearingValue);
	//printf("  MyGoal = %.f,", myGoal);
	printf("  Sudut = %.f, \n", angle);
	printf("  Sudut Offset = %.f, \n", strategy.offset);
	//printf("  Sudut Tambah = %lf, \n", suTam);
	//printf("  Sudut Akhir = %lf, \n", suHir);
	//printf("  Sudut Kalkulasi = %.f, \n", odometry.ArchSin);
	printf("  Sudut OutGrid = %.f, \n", outGrid);
	//printf("  Odometry Robot Koordinat X = %.2lf\n", robotPos_X);
	//printf("  Odometry Robot Koordinat Y = %.2lf\n", robotPos_Y);
	//---------------------------------------------------------
	//sudut2();
	//printf("  Bearing2 = %.f", bearingValue2);
	//printf("  MyGoal2 = %.f,", setPointAngle);
	//printf("  Sudut2 = %.f,", angle2);
	//---------------------------------------------------------
	printf("\n  posPan = %.2f,  posTilt = %.2f,", posPan, posTilt);
	//printf("  lastPan = %.2f,  lastTilt = %.2f,", ballPan, ballTilt);
	//printf("  searchKe = %.f,", searchKe);
	//printf("  Matte = %d,", matte);
	//printf("  semeh = %d,", semeh);
	//---------------------------------------------------------
	printf("\n  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot1Id, koordinasi.robot1FBall, koordinasi.robot1DBall, koordinasi.robot1State, koordinasi.robot1Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot2Id, koordinasi.robot2FBall, koordinasi.robot2DBall, koordinasi.robot2State, koordinasi.robot2Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot3Id, koordinasi.robot3FBall, koordinasi.robot3DBall, koordinasi.robot3State, koordinasi.robot3Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot4Id, koordinasi.robot4FBall, koordinasi.robot4DBall, koordinasi.robot4State, koordinasi.robot4Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot5Id, koordinasi.robot5FBall, koordinasi.robot5DBall, koordinasi.robot5State, koordinasi.robot5Status);
	if (useCoordination){
		printf("  BarelangFC%d : Tengah = %d, \t Kanan = %d, \t Kiri = %d, \n", robotID, balikTengah, balikSampingKanan, balikSampingKiri);
	} else {
		printf("  BarelangFC%d : Tengah = %d, \t Kanan = %d, \t Kiri = %d, \n", robotID, lockMidBack, lockRightBack, lockLeftBack);
	}
	//---------------------------------------------------------
	//getServPos();
	//printf("\n  head(%.lf, %.lf),", head1, head2);
	//printf("  Larm(%.lf, %.lf, %.lf),", Larm1, Larm2, Larm3);
	//printf("  Rarm(%.lf, %.lf, %.lf),", Rarm1, Rarm2, Rarm3);
	//printf("  Lleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf),", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
	//printf("  Rleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf),\n", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
	//---------------------------------------------------------
	//kalkulasiJarakBola();
	//printf("\n  jarakBola = %.f,", jarakBola);
	//printf("  jarakBola_Y = %.f,", jarakBola_Y);
	//printf("  jarakBola_X = %.f,\n", jarakBola_X);
	//hitungGerakBola();
	//printf("  deltaY = %.f,", deltaY);
	//printf("  deltaX = %.f,", deltaX);
	//printf("  gradient = %.f,", gradient);
	//---------------------------------------------------------
	//printf("  countDribble = %d,", countDribble);
	//---------------------------------------------------------
	//printf("  data motion = %s,", dataMotion);
	//printf("  data head = %s,", dataHead);
	//---------------------------------------------------------
	//Robocup GameController Data
	//printf("\n\n  Game State = %d,", gameController.State);
	//printf("\n  Version = %d,", gameController.Version);
	//printf("\n  packetNumber = %d,", gameController.PacketNumber);
	//printf("\n  playersPerTeam = %d,", gameController.PlayerTeam);
	//printf("\n  gameType = %d,", gameController.GameTipe);
	//printf("\n  firstHalf = %d,", gameController.FirstHalf);
	//printf("\n  kickOffTeam = %d,", gameController.KickOff);
	//printf("\n  secondaryState = %d,", gameController.SecondaryState);
	//printf("\n  dropInTeam = %d,", gameController.DropTeam);
	//printf("\n  dropInTime = %d,", gameController.DropTime);
	//printf("\n  secsRemaining = %d,", gameController.Remaining);
	//printf("\n  secondaryTime = %d,", gameController.SecondaryTime);

	//Team Info left
	//printf("\n\n  TeamNumber = %d,", gameController.timNumber1);
	//printf("\n  teamColour = %d,", gameController.timColour1);
	//printf("\n  score = %d,", gameController.Score1);
	//printf("\n  penaltyShot = %d,", gameController.Penaltyshoot1);
	//printf("\n  singleShots = %d,", gameController.Singleshoot1);
	//printf("\n  coachSequence = %d,", gameController.Coachsequence1);

	//Team Info Right
	//printf("\n\n  TeamNumber = %d,", gameController.timNumber2);
	//printf("\n  teamColour = %d,", gameController.timColour2);
	//printf("\n  score = %d,", gameController.Score2);
	//printf("\n  penaltyShot = %d,", gameController.Penaltyshoot2);
	//printf("\n  singleShots = %d,", gameController.Singleshoot2);
	//printf("\n  coachSequence = %d,", gameController.Coachsequence2);

	//Robot Info Left
	//printf("\n\n  penalty = %d,", gameController.Penalty1);
	//printf("\n  secsTillUnpenalised = %d,", gameController.TimeUnpenalis1);
	//printf("\n  yellowCardCount = %d,", gameController.YellowCard1);
	//printf("\n  redCardCount = %d,", gameController.RedCard1);

	//Robot Info Right
	//printf("\n\n  penalty = %d,", gameController.Penalty2);
	//printf("\n  secsTillUnpenalised = %d,", gameController.TimeUnpenalis2);
	//printf("\n  yellowCardCount = %d,", gameController.YellowCard2);
	//printf("\n  redCardCount = %d,", gameController.RedCard2);

	//printf("\n ---------------PARAMETER LOKALISASI---------------\n");
	printf("\n  Robot Koordinat X = %.2lf\n", robotPos_X);
	printf("  Robot Koordinat Y = %.2lf\n", robotPos_Y);
	//printf("  Odometry Rotate  = %.2lf\n", odometry.ArchSin);
	printf("\n");
}


void setting() {
//========================================================================
	usePenaltyStrategy	= false; //untuk pinalti
	useVision		= true;
	useSocket		= true;
	useImu			= true;
	useAutoDirection	= false;
	useGameController	= true;
	useCoordination		= true;
	useLocalization		= true;
	useUpdateCoordinate	= false; //true;
	useFollowSearchGoal	= true;
	useSearchGoal		= false;
	useDribble		= false;
	dribbleOnly		= false;
	useSideKick		= true;
	useLastDirection	= true;
	useNearFollowSearchGoal	= false;
	firstStateCondition	= 0; //-2; //0;
//========================================================================
}

/////////////////////////////////////////////////////////
/////////////////// Main_Program ////////////////////////
/////////////////////////////////////////////////////////
int main(void) {
	//semahau(robotID); //robot parameters
	loadConfig(); //robot parameters
	setting();

	//open socket for head&motion
	if (useSocket) {
		initSendDataMotion();
		initSendDataHead();
	}

	//open socket for lokalisasi
	initSendDataLokalisasi();

	//open socket for monitoring
	monitor.initSendMonitor();


	runLuaProgram();
	motion("8"); //robot berdiri

	pthread_create(&threadMonitor,NULL, sendMonitoring, NULL);
	pthread_create(&threadStrategy,NULL, updateStrategy, NULL);
	//pthread_create(&threadVision, NULL, camera, NULL);
	//pthread_create(&threadPID, NULL, camera, NULL);
	pthread_create(&threadReceiverVision, NULL, receiverVision, NULL);
	pthread_create(&threadReceiverLocalization, NULL, receiverLocalization, NULL);
	pthread_create(&threadGC_Read, NULL, gcReadData, NULL);
	pthread_create(&threadGC_Return, NULL, gcReturnData, NULL);
	pthread_create(&Lokalisasi, NULL, sendData, NULL);

	//GameController
	gameController.Player	= robotID;
	gameController.Team	= TEAM;
	gameController.Server	= SERVER;

	//Coordination
	koordinasi.RobotID	= robotID;

	//communications
	if (useCoordination) {
		koordinasi.initCommunicationIn();
		koordinasi.initCommunicationOut();
	}

	/////////////////////////////////
	////////// first doing //////////
	/////////////////////////////////
	motion("0"); //robot diam
	headMove(0.0, -1.2);

	while(1) {
		usleep(40000);
		monitor.sendMonitor( robotID, robotPos_X, robotPos_Y, strategy.bearingValue, outGrid, stateCondition);
		display();
//		displayMoveLokalisasi();
		getImuSensor();
		getSensor();
//		arah();
		sudut();
		cekArah();

		if (useLocalization){
			if (strategy.strategyNumber != 15) {
				getServPos();
				gridCoor();
				odometry.bacaKondisi(odometry.RobotWalk_X, odometry.RobotWalk_Y);

				robotPos_X = odometry.deltaPos_X + initialPos_X;
				robotPos_Y = odometry.deltaPos_Y + initialPos_Y;

				odometry.RobotPos_X = robotPos_X;
				odometry.RobotPos_Y = robotPos_Y;
				odometry.trigonoMetri();
			}
		} else { followSearchAktif = true; }

		if (useLastDirection == true && lastDirection != 0) {
			if ((angle < lastDirection+15) && (angle > lastDirection-15)) {
				useImu = false;
				//robotDirection = true;
			} else {
				lastDirection = 0;
				useImu = true;
				//robotDirection = true;
			}
		}

		/*
		//INTIAL POSITION NASIONAL
		if(useGameController) {
			if(!play && gameController.State == 0 && !usePenaltyStrategy || switched) {
				if ((!pickUp && gameController.Remaining == 600 &&  gameController.SecondaryState == 0) || (!pickUp && gameController.Remaining == 300 &&  gameController.SecondaryState == 2) || switched) {
					if (strategy.strategyNumber <=4 || strategy.strategyNumber == 13) {	//Initial nilai odometry masuk tengah
						if (countInitPos == 0) {
							initialPos_X = -358;
							initialPos_Y = 0;
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						}
					} else {								//Initial nilai odometry masuk samping
						if ( angle >= 0 && countInitPos == 0) {				//Support Kanan
							initialPos_X = -358;
							initialPos_Y = 130;
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						} else if (angle < 0 && countInitPos == 0) {			//Support Kiri
							initialPos_X = -358;
							initialPos_Y = -130;
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						}

					}
				}
			} else if (play && !usePenaltyStrategy && !switched) {
				if ((pickUp && gameController.Remaining != 600 && gameController.SecondaryState == 0 && !switched) || (pickUp && gameController.Remaining != 300 && gameController.SecondaryState == 2 && !switched)) {
					if (angle >= 0 && countInitPos == 0) {
						initialPos_X = -75;
						initialPos_Y = -305;
					} else if (angle < 0 && countInitPos == 0) {
						initialPos_X = -75;
						initialPos_Y = 305;
					}
				}
			} else if (!play && gameController.State == 0 && usePenaltyStrategy) {
				initialPos_X = 150;
				initialPos_Y = 0;
			}
		} else {
			if (!play && !usePenaltyStrategy){
				initialPos_X = -350;
				initialPos_Y = 0;
			} else if (!play && usePenaltyStrategy) {
				initialPos_X = 150;
				initialPos_Y = 0;
			}
		}
		*/


		//INTIAL POSITION INTERNASIONAL
		if(useGameController) {
			if(!play && gameController.State == 0 && !usePenaltyStrategy || switched) {
				if ((!pickUp && gameController.Remaining == 600 &&  gameController.SecondaryState == 0) || (!pickUp && gameController.Remaining == 300 &&  gameController.SecondaryState == 2) || switched) {
					if (strategy.strategyNumber <=4 || strategy.strategyNumber == 13) {	//Initial nilai odometry masuk tengah
						if (countInitPos == 0) {
							if (angle >= 0) {
								initialPos_X = -130;
								initialPos_Y = -300;
							} else if (angle < 0) {
								initialPos_X = -130;
								initialPos_Y = 300;
							}
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						}
					} else {								//Initial nilai odometry masuk samping
						if ( angle >= 0 && countInitPos == 0) {				//Support Kanan
							initialPos_X = -300;
							initialPos_Y = -305;
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						} else if (angle < 0 && countInitPos == 0) {			//Support Kiri
							initialPos_X = -300
							;
							initialPos_Y = 305;
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						}

					}
				}
			} else if (play && !usePenaltyStrategy && !switched) {
				if ((pickUp && gameController.Remaining != 600 && gameController.SecondaryState == 0 && !switched) || (pickUp && gameController.Remaining != 300 && gameController.SecondaryState == 2 && !switched)) {
					if (angle >= 0 && countInitPos == 0) {
						initialPos_X = -300;
						initialPos_Y = -305;
					} else if (angle < 0 && countInitPos == 0) {
						initialPos_X = -300;
						initialPos_Y = 305;
					}
				}
			} else if (!play && gameController.State == 0 && usePenaltyStrategy) {
				initialPos_X = 240;
				initialPos_Y = 0;
			}
		} else {
			if (!play && !usePenaltyStrategy){
				initialPos_X = -350;
				initialPos_Y = 0;
			} else if (!play && usePenaltyStrategy) {
				initialPos_X = 240;
				initialPos_Y = 0;
			}
		}


		if (useCoordination) {
			refreshComm(); //refresh data communications


			if (zeroState == true || play == false || stateCondition == 1 || stateCondition == 51 || stateCondition == 150 || stateCondition == 200) {
				countHilang = 0;
				countDapat = 0;
				semeh = 232; koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus);
			} else if (stateCondition == 7 || stateCondition == 3 || stateCondition == 8 || stateCondition == 4 || stateCondition == 5 || stateCondition == 52 || stateCondition == 53 || stateCondition == 90 || stateCondition == 100 || stateCondition == -10 || stateCondition == 10 || stateCondition == 20 || stateCondition == 30 || stateCondition == 40) {
				countHilang = 0;
				countDapat = 0;
				semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, stateCondition, robotStatus);
			} else if (stateCondition == 2 || stateCondition == 6) {
				//langsung kirim didalam case
			} else { //sisa stateCondition yang tidak dikondisikan
				if (receler.Ball_X == -1 && receler.Ball_Y == -1) { //bola hilang
					countDapat = 0;
					if (countHilang > 20) {
						semeh = 232;
						koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus);
					} else {
						countHilang++;
					}
				} else { //dapat bola
					countHilang = 0;
					trackBall();
					koordinasi.sendRobotCoordinationData(robotID, 1, semeh, stateCondition, robotStatus);
					//if (countDapat <= 20) {
					//	semeh = 232;
					//	countDapat++;
					//}
				}
			} koordinasi.readRobotCoordinationData();
		}

		if (strategy.strategyNumber == 15) { //printf("  check receler\n");
			motion("0");
			resetAllVariable();

			getSensor();
			setPointAngle = angleZ;

			gameController.State = 0;
			stateGameController = 0;
			lastStateGameController = 0;

			play	= false;

			pickUp	= false;
			trigger	= false;
			//firstTimes = true;
			manual = true;
			countInitPos = 0;

			robotStatus = 0;

			if (masha < 50) {
				predictGoal(angle, -1.6);
				masha++;
			} else {
				if (ballLost(20)) {
					tiltSearchBall(0.0);
				} else {
					trackBall();
				}
			} stateCondition = 150;
		} else {
			if (state == 0) robotStatus = 1;
			masha = 0;
		}

		if ((gameController.Penalty1 == 34 && gameController.timNumber1 == BARELANG_COLOR) || (gameController.Penalty2 == 34 && gameController.timNumber2 == BARELANG_COLOR)) {
			zeroState = true;
			signIn = false;
		} else {
			signIn = true;
		}

		if (stateGameController == 3) {
			if (robotFall == true) {
				resetCase1();
				stateCondition = 1;
			}
		}

//		if (strategy.bearingValue == 0) {
//			countBearing += 1;
//			if (countBearing >= 200) {
//				stateCondition = 200;
//			}
//		} else {
//			countBearing = 0;
//		}


		////////////////////////////////////////////////////////////////////////
		////////////////.............Game Controller.............///////////////
		////////////////////////////////////////////////////////////////////////
		if (useGameController) {
			stateGameController = gameController.State;
//			if (lastStateGameController != stateGameController) {
//				if (stateGameController > lastStateGameController) { //printf("maju\n");
//					if (stateChange > 30) { //40
//						lastStateGameController = stateGameController;
//					} else { stateChange++; }
//				} else if (stateGameController < lastStateGameController) { //printf("mundur\n");
//					if (
//						(lastStateGameController == 4 && stateGameController == 0) ||
//						(lastStateGameController == 3 && stateGameController == 1)
//					   ) {
//						if (stateChange > 30) { //40
//							lastStateGameController = stateGameController;
//						} else { stateChange++; }
//					} //else {
					//	if (stateChange > 60) { //40
					//		lastStateGameController = stateGameController;
					//	} else { stateChange++; }
					//}
//				}
//			} else { stateChange = 0; }


			/*kickOff = gameController.KickOff;
			if (lastKickOff != kickOff) {
				if (kickOffChange > 40) { //30
					lastKickOff = kickOff;
				} else { kickOffChange++; }
			} else { kickOffChange = 0; }


			secRemaining = gameController.Remaining;
			if (lastSecRemaining != secRemaining) {
				if (secRemainingChange > 40) { //30
					lastSecRemaining = secRemaining;
				} else { secRemainingChange++; }
			} else { secRemainingChange = 0; }*/


			//switch (lastStateGameController) {
			switch (stateGameController) {
				case 0 : //printf("  Initial\n\n");
					zeroState = false;
					play = false;

					if (strategy.strategyNumber != 15) {
						motion("0"); //Walk(0.0, 0.0, 0.0);
						//headMove(0.0, -1.1);
						//sudut();
						predictGoal(angle, -1.6);

						backPosition = false;
						resetAllVariable();

						stateCondition = 150;		

						//posRotate = false;
						//setWaktu();
					}
				break;

				case 1 : //printf("  Ready\n\n\n");
					zeroState = false;
					play = false;

					if (kurama == 0) {
						reset = 0;
						manual = false;
						kurama = 1;
					}

					if (strategy.strategyNumber != 15) {
						//mode1...
							//motion("0");
							//resetAllVariable();
							//if (ballLost(20)) {
								//sudut();
								//predictGoal(angle, -1.6);
							//	panSearchBall(-0.8);
							//} else {
							//	trackBall();
							//}
							//stateCondition = 150;		

						//mode2...
							if (manual) {
								motion("0");
								resetAllVariable();
								predictGoal(angle, -1.6);
								//if (ballLost(20)) {
									//sudut();
									//predictGoal(angle, -1.6);
								//	panSearchBall(-0.8);
								//} else {
								//	trackBall();
								//}
								backPosition = false;
							} else {
								localization();
							}
							Move = true;
							stateCondition = 150;		

						//mode3...
							//motion("9");
							//SearchBall(2);
							//cekWaktu(15); //27
							//if (timer) {
							//	if (posRotate) {
							//		//motion("0");
							//		//jalanDirection(0.02, 0.0, 0);
							//		Walk(0.02, 0.0, 0.0);
							//	} else {
							//		rotateBodyImu(0);
							//	}
							//} else {
							//	//jalanDirection(jalan, 0.0, -90);
							//	Walk(jalan, 0.0, 0.0);
							//}
							//stateCondition = 150;		
					}
				break;

				case 2 : //printf("  \n\n\nSet\n\n\n\n");
					zeroState = false;
					play = false;
					kurama = 0;
					resetKoordinasiRobotBalik();
					if (strategy.strategyNumber != 15) {
						motion("0");

						firstTimes	= true;
						pickUp		= false;
						trigger		= false;
						switched	= false;
						kondisiBola	= 0;
						stateCondition	= 0;
						reset		= 0;
						countDef	= 0;

						resetCaseAwal();
						//resetAllVariable();
						stateCondition = 150;			

						if (ballLost(20)) {
							SearchBall(2);
						} else {
							trackBall();
						}
					}
				break;

				case 3 : //printf("play\n");
					kurama = 0;
					if (strategy.strategyNumber != 15) {
						if (firstTimes) {
							//motion("9");
							//setWaktu();			

							searchKe = 0;
							timer = false;
							resetCaseAwal();
							//resetAllVariable();

							stateCondition = firstStateCondition;
							firstTimes = false;
							play = true;
						}
						//display();				
						//play = true;
					}
				break;

				case 4 : //printf("finish\n");
					zeroState = false;
					kurama = 0;
					motion("0");
					motion("7");
					//headMove(0.0, -1.2);
					//sudut();
					predictGoal(angle, -1.6);

					play		= false;
					manual		= false;
					backPosition	= false;
					resetAllVariable();
					stateCondition = 150;
				break;

				default: break;
			}
		} else {
			if (firstTimes) {
				backPosition = false;
				resetAllVariable();
				stateCondition = firstStateCondition;
				firstTimes = false;
			}
			play = true;
		}

		////////////////////////////////////////////////////////////////////////
		////////////////.............Strategy Button.............///////////////
		////////////////////////////////////////////////////////////////////////
		//state = strategy.killnRun;
		if (strategy.killnRun == 1) { //printf("killall\n")
			if (wait < 20) { wait++; }
			else {
				state = 1;
				if (stateGameController == 3 || lastStateGameController == 3) { zeroState = true; }
			}
		} else { //printf("state run lua\n");
			wait = 0;
			state = 0;
		}

		//printf("wait = %d, state = %d\n",wait,state);

		if (lastState != state) {
			if (state == 1) { //printf("killall\n");
				system("killall screen;");
				resetAllVariable();
				resetOdometry();
				refreshRecelerLokalisasi();

				stateCondition = 150;
				countInitPos = 0;
				robotStatus = 0;

				pickUp = false;
				gameController.Penalise = false;
			} else if (state == 0) { //printf("state run lua\n");
				runLuaProgram();
				motion("8");

				setting();
				zeroState = false; //didnt find ball for coor
				backPosition = false;
				kurama = 0;
				manual = false;
				reset = 0;
				robotStatus = 1;

				if (stateGameController == 1) {
					switched = true;
				} else {
					switched = false;
				}

				gameController.State = 3;
				stateGameController = 3;
				lastStateGameController = 3;

				gameController.Penalise = true;

				play = true;

				pickUp = true;
				stateCondition = firstStateCondition;
			} lastState = state;
		}

		// hold button
/*		state2 = strategy.reset;
		if (lastState2 != state2) {
			if (state2 == 1) {
				motion("0");

				headMove(0.0, -1.6);
				resetAllVariable();
				zeroState = true;
				stateCondition = 150;

				gameController.Penalise = false;
			} else if (state2 == 0) {
				motion("8");

				setting();
				zeroState = false;
				backPosition = false;
				kurama = 0;
				manual = false;
				reset = 0;
				countInitPos = 0;

				if (stateGameController == 1) {
					switched = true;
				} else {
					switched = false;
				}

				gameController.State = 3;
				stateGameController = 3;
				lastStateGameController = 3;

				gameController.Penalise = true;

				play = true;

				pickUp = true;
				stateCondition = firstStateCondition;
			} lastState2 = state2;
		}
*/

		///////////////////////////////////////////////////////////////////////
		//////////////.............Role of execution............///////////////
		///////////////////////////////////////////////////////////////////////
		if (play) { //printf("BarelangFC Rock n Roll\n");
			switch(stateCondition) {
				case 0: // First Strategy
					if (!backPosition) {
						Strategi = strategy.strategyNumber;
					} else {
						if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //ketika dikiri dan sebagai attack
						//if (gameController.KickOff == TEAM || gameController.KickOff == 128) { //ketika dikiri dan sebagai attack
							Strategi = 0; //printf("Serang...\n");
						} else { //ketika dikiri dan sebagai defense
							Strategi = 13; //printf("Defense...\n");
						}
					}

					switch (Strategi) {
					//switch (strategy.strategyNumber;) {
						case 0 : // serang lurus baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

//								if (angle > 0) {
//									panRate		= 0.05;
//									tiltRate	= -0.05;
//								} else {
//									panRate		= -0.05;
//									tiltRate	= -0.05;
//								}

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 51;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									if (!usePenaltyStrategy) {
										useFollowSearchGoal = true;
										useSearchGoal	= false;
									} else { //Ketika Pinalti
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;

								stateCondition	= -10; //-10; //-20;
							}
						break;

						case 1 : // serang ke kanan baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

//								if (angle > 0) {
//									panRate		= 0.05;
//						        		tiltRate	= -0.05;
//								} else {
//									panRate		= -0.05;
//									tiltRate	= -0.05;
//								}

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 51;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									if (!usePenaltyStrategy) {
										useFollowSearchGoal = true;
										useSearchGoal	= false;
									} else { //Ketika Pinalti
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;

								stateCondition	= 10;
							}
						break;

						case 2 : // serang ke kiri baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

//								if (angle > 0) {
//									panRate		= 0.05;
//					                        	tiltRate	= -0.05;
//								} else {
//									panRate		= -0.05;
//					                        	tiltRate	= -0.05;
//								}

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 51;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									if (!usePenaltyStrategy) {
										useFollowSearchGoal = true;
										useSearchGoal	= false;
									} else { //Ketika Pinalti
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;

								stateCondition	= 20;
							}
						break;

						case 3 : // serang ke kanan tanpa baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

//								if (angle > 0) {
//									panRate		= 0.05;
//		        			                	tiltRate	= -0.05;
//								} else {
//									panRate		= -0.05;
//					                        	tiltRate	= -0.05;
//								}

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 51;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

								stateCondition	= 30;
							}
						break;

						case 4 : // serang ke kiri tanpa baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

//								if (angle > 0) {
//									panRate		= 0.05;
//		        			                	tiltRate	= -0.05;
//								} else {
//									panRate		= -0.05;
//		                        				tiltRate	= -0.05;
//								}

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 51;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

								stateCondition	= 40;
							}
						break;

						case 5 : // pickup dribble baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= true;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= true;
							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 50;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 6 : // pickup dribble tanpa baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= true;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= true;
							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 50;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 7 : // pickup tanpa dribble baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 50;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 8 : // pickup tanpa dribble tanpa baca gawang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;
							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 50;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 9 : // pickup prioritas
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							}

//							if (angle > 0) {
//								panRate		= 0.05;
//		       			                	tiltRate	= -0.05;
//							} else {
//								panRate		= -0.05;
//		                       				tiltRate	= -0.05;
//							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 53; //100;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 10 : // pickup prioritas
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							}

//							if (angle > 0) {
//								panRate		= 0.05;
//		       			                	tiltRate	= -0.05;
//							} else {
//								panRate		= -0.05;
//		                       				tiltRate	= -0.05;
//							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 53; //90;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 11 : // pickup prioritas
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							}

//							if (angle > 0) {
//								panRate		= 0.05;
//		       			                	tiltRate	= -0.05;
//							} else {
//								panRate		= -0.05;
//		                       				tiltRate	= -0.05;
//							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 53; //100;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 12 : // pickup prioritas
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							} else {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;
							}

//							if (angle > 0) {
//								panRate		= 0.05;
//		       			                	tiltRate	= -0.05;
//							} else {
//								panRate		= -0.05;
//		                       				tiltRate	= -0.05;
//							}

							if (signIn) {
								motion("9");
								setWaktu();
								stateCondition = 53; //90;
							} else {
								motion("0");
								headMove(0.0, -1.4);
							}
						break;

						case 13 : // Defense didepan
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 52;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("0");

								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;

								setWaktu();
								stateCondition	= 130;
							}
						break;

						case 14 : // Defense dibelakang
							if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
								trackBall();
							}

							if (pickUp) {
								if (useLocalization) {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = false;
									useSearchGoal	= false;
								}
								//useImu	= false;
								useDribble	= false;

								if (signIn) {
									motion("9");
									setWaktu();
									stateCondition = 52;
								} else {
									motion("0");
									headMove(0.0, -1.4);
								}
							} else {
								motion("9");

								if (useLocalization) {
									useFollowSearchGoal = true;
									useSearchGoal	= false;
								} else {
									useFollowSearchGoal = true;
									useSearchGoal	= true;
								}
								//useImu	= false;
								useDribble	= false;

								setWaktu();
								stateCondition	= 140;
							}
						break;

						case 15 : // cek receler
							stateCondition = 150;
						break;

						default:
						break;
					}
				break;

				case 1: // searching ball
					motion("9");

					if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
						delayWaitBall = 0;
						normalSearchBall();
					} else {
						tracked = true;
					}

					if (tracked) {
						if (ballLost(20)) {
							tracked = false;
						} else {
							trackBall();

							if (delayWaitBall > 30) { //30
								if (headTilt >= -1.8 && headPan >= -0.1 && headPan <= 0.1) {
									resetCase2();
									stateCondition = 2;
								} else {
									followBall(0);
								}
							} else {
								Walk(0.0, 0.0, 0.0);
								delayWaitBall++;
							} //printf("  delayWaitBall = %d,", delayWaitBall);
						}
					}
				break;

				case 2: // follow, search goal dan coor
					motion("9");

					if (useCoordination) {
						if (robotID == 1) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 ||koordinasi.robot4State == 53) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
							   ) {
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 2) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
							   ) {
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 3) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
							   ) {
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 4) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == 90 || koordinasi.robot5State == 100 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30 || koordinasi.robot5State == 40 || koordinasi.robot5State == 52 || koordinasi.robot5State == 53)
							   ) {
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 5) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == 90 || koordinasi.robot1State == 100 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30 || koordinasi.robot1State == 40 || koordinasi.robot1State == 52 || koordinasi.robot1State == 53) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == 90 || koordinasi.robot2State == 100 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30 || koordinasi.robot2State == 40 || koordinasi.robot2State == 52 || koordinasi.robot2State == 53) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == 90 || koordinasi.robot3State == 100 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30 || koordinasi.robot3State == 40 || koordinasi.robot3State == 52 || koordinasi.robot3State == 53) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == 90 || koordinasi.robot4State == 100 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30 || koordinasi.robot4State == 40 || koordinasi.robot4State == 52 || koordinasi.robot4State == 53)
							   ) {
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall)
							   ) {
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						}
					} else {
						myTurn();
					}
				break;

				case 7: // imu
					motion("9");

					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} tunggu++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						tunggu = 0;

						if (useImu) {
							if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
								//if (headTilt >= (cSekarang - 0.2)) {
									Walk(0.0, 0.0, 0.0);

									//if (gridX == 9 && useLocalization == true) { // ada di daerah depan gawang (sudah tidak bisa lagi untuk baca gawang), opsinya adalah tendang langsung dengan estimasi angle lokalisasi
									//	resetCase5();
									//	stateCondition = 5;
									//} else {
										if (useDribble == true || useSearchGoal == true) {
											resetCase3();
											stateCondition = 3;
										} else {
											resetCase5();
											stateCondition = 5;
										}
									//}
								//} else {
								//	if (headTilt >= -1.0) {						
								//		ballPositioning(0.0, cSekarang, 0.12);
								//	} else {
								//		followBall(0);
								//	}
								//}
							} else {
								if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
									if (delay > 5) {
										cekWaktu(20);
										if (timer) {
											robotDirection = true;
										} else {
											if (useSideKick) {
												if (useLocalization) {
													if (robotPos_X <= -225) {
														if (angle >= 50) { //45
															useSearchGoal = false;
															modeKick = 4; //tendangSamping
															Imu(90, cSekarang); //90
														} else if (angle <= -50) { //-45
															useSearchGoal = false;
															modeKick = 3; //tendangSamping
															Imu(-90, cSekarang); //-90
														} else {
															useSearchGoal = false;
															modeKick = tendangJauh;
															//Imu(sudutTengah + estimasiAngle, cSekarang);
															Imu(0, cSekarang);
														}
													} else if (robotPos_X > -225 && robotPos_X < 225) {
														if (angle >= 50) { //45
															useSearchGoal = false;
															modeKick = 4; //tendangSamping
															Imu(90 + odometry.ArchSin, cSekarang); //90
														} else if (angle <= -50) { //-45
															useSearchGoal = false;
															modeKick = 3; //tendangSamping
															Imu(-90 + odometry.ArchSin, cSekarang); //90
														} else {
															useSearchGoal = false;
															modeKick = tendangJauh;
															Imu(odometry.ArchSin, cSekarang);
														}
													} else if (robotPos_X >= 225) {
														if (robotPos_Y > 0) { // posisi Y dari 0 - 300
															if (angle >= 20) { //45
																useSearchGoal = false;
																modeKick = 4; //tendangSamping
																Imu(90 + outGrid, cSekarang); //90
															} else {
																useSearchGoal = false;
																modeKick = tendangJauh;
																Imu(outGrid, cSekarang);
															}
														} else { // posisi Y dari -300 - 0
															if (angle <= -20) { //-45
																useSearchGoal = false;
																modeKick = 3; //tendangSamping
																Imu(-90 + outGrid, cSekarang); //90
															} else {
																useSearchGoal = false;
																modeKick = tendangJauh;
																Imu(outGrid, cSekarang);
															}
														}
													}
												} else {
													if (angle >= 50) { //45
														useSearchGoal = false;
														modeKick = 4; //tendangSamping
														Imu(75, cSekarang); //90
													} else if (angle <= -50) { //-45
														useSearchGoal = false;
														modeKick = 3; //tendangSamping
														Imu(-75, cSekarang); //-90
													} else {
														useSearchGoal = false;
														modeKick = tendangJauh;
														//Imu(sudutTengah + estimasiAngle, cSekarang);
														Imu(0, cSekarang);
													}
												}
											} else {
												modeKick = tendangJauh;
												if (useLocalization){
													//Imu(odometry.ArchSin, cSekarang);
													Imu(outGrid, cSekarang);
												} else {
													//Imu(sudutTengah + estimasiAngle, cSekarang);
													Imu(0, cSekarang);
												}
											}
										}
									} else {
										setWaktu();
										robotDirection = false;
										delay++;
									}
								} else {
									delay = 0;
									followBall(0);
								}
							}
						} else {
							if (headTilt >= (cSekarang - 0.2)) {
								Walk(0.0, 0.0, 0.0);

								//if (gridX == 9 && useLocalization == true) { // ada di daerah depan gawang (sudah tidak bisa lagi untuk baca gawang), opsinya adalah tendang langsung dengan estimasi angle lokalisasi
								//	resetCase5();
								//	stateCondition = 5;
								//} else {
									if (useDribble == true || useSearchGoal == true) {
										resetCase3();
										stateCondition = 3;
									} else {
										resetCase5();
										stateCondition = 5;
									}
								//}
							} else {
								followBall(0);
							}
						}
					}
				break;

				case 3: // dribble
					motion("9");

					if(ballLost(20)) {
						if (tunggu > 40) {
							useDribble = false;
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} tunggu++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						tunggu = 0;					

						if (useDribble) {
							if (countDribble > 300) { //300 //printf("ke state SearchGoal....................................................................\n\n\n");
								if (delay > 30) {
									resetCase8();
									stateCondition = 8;
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							} else {
								//dribble(sudutTengah, 0.15); //0.14 //0.05 //0.08 //arah kompas, speed; speed = speed * 0.3
								dribble(outGrid, 0.15); //0.14 //0.05 //0.08 //arah kompas, speed; speed = speed * 0.3
								countDribble++;
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							
							if (useSearchGoal == true) {
								resetCase4();
								stateCondition = 4;
							} else {
								resetCase5();
								stateCondition = 5;
							}
						}
					}
				break;

				case 8: //Imu lagi setelah dribble
					motion("9");

					if(ballLost(20)) {
						if (tunggu > 40) {
							useDribble = false;
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						//sudut();
						tunggu = 0;

						if (useImu) {
							if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
								Walk(0.0, 0.0, 0.0);		
								resetCase4();
								stateCondition = 4;
							} else {
								if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
									modeKick = tendangJauh;
									//Imu(sudutTengah + estimasiAngle, cSekarang);
									Imu(outGrid, cSekarang);
								} else {
									robotDirection = false;
									followBall(0);
								}
							}
						} else {
							if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
								Walk(0.0, 0.0, 0.0);			
								resetCase4();
								stateCondition = 4;
							} else {
								followBall(0);
							}
						}
					}
				break;

				case 4: // Search Goal
					motion("9");					

					if (useSearchGoal) { //printf("cariGawang\n\n");
						if (rotateGoal == true) {
							Walk(0.0, 0.0, 0.0);				
							if (dribbleOnly) {
								useDribble = true;
								resetCase3();
								stateCondition = 3; //dribble
							} else {
								resetCase5();
								stateCondition = 5; //kick
							}
						} else {
							rotateToGoal(1); //1
						}
					} else { //printf("tidakCariGawang\n\n");
						Walk(0.0, 0.0, 0.0);					
						if (dribbleOnly) {
							useDribble = true;
							resetCase3();
							stateCondition = 3; //dribble
						} else {
							resetCase5();
							stateCondition = 5; //kick
						}
					}
				break;

				case 5: // Kick
					if (ballLost(30)) {
						if (tunggu > 40) {
							useDribble = false;
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();

						if (tendang) {
							//Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							//if (tendangLagi == true || (headPan >= -0.6 && headPan <= 0.6)) {
								//kick(tendangJauh);
								kick(modeKick);
							//} else {
							//	resetCase2();
							//	stateCondition = 2;
							//}
						}
					}
				break;

				case 6: // Search After Kick
					motion("9");

					if (tunggu > 5) { //kedua
						cekWaktu(3); //4

						if (timer) {
							if (ballLost(20)) {
								confirmsBall = 0;
								waitTracking = 0;

								if (searchRectangle) {
									if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, 0, semeh, stateCondition, robotStatus); }

									if (sumTilt > 300) {
										if (Strategi >= 0 && Strategi < 3 || Strategi >= 9 && Strategi <= 13) {
											setting();
										}
										resetCase1();
										stateCondition = 1;
									} else {
										searchBallRectang2(-1.5, -1.6, -0.8, 1.6);
										//tiltSearchBall(0.0); // second Search
										sumTilt++;
									}

									if (modeKick == 3 || modeKick == 4) { //sideKick
										jalanDirection(kejar, 0.0, lastDirection); //saveAngle				
									} else {
										Walk(kejar, 0.0, 0.0);
									}
								} else {
									if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, 232, robotStatus); }
									Walk(0.0, 0.0, 0.0);					

									if (tunda > 10) {
										SearchBall(3); //first Search
										if (posPan >= batasKiri-0.1 && countTilt == 1) {
											tiltKe = countTilt;
											searchRectangle = true;
										}
									} else {
										posPan  = batasKiri;
										posTilt = batasBawah;
										tiltRate1 = -0.06;
										tiltRate2 = -0.08;
										tiltRate3 = -0.1;
										panRate1 = -0.06;
										panRate2 = -0.08;
										panRate3 = -0.1;
										headMove(posPan, posTilt);
										tunda++;
									}
								}
							} else {
								trackBall();

								if (waitTracking > 20) {
									searchRectangle = true;
								} else {
									waitTracking++;
								}

								if (useCoordination) {
									koordinasi.sendRobotCoordinationData(robotID, 1, semeh, stateCondition, robotStatus);
									backToCoordinations();
								}

								if (confirmsBall > 30) {
									//if (headTilt >= -0.8 && headPan >= -0.6 && headPan <= 0.6 && searchRectangle == false) { //langsung Tendang lagi
									if (headTilt >= -0.7 && searchRectangle == false) { //langsung Tendang lagi
										Walk(0.0, 0.0, 0.0);
										//tendangLagi = true;				
										resetCase5();
										stateCondition = 5;
									} else {
										//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
										//if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
										if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
											if (Strategi >= 0 && Strategi < 3 || Strategi >= 9 && Strategi <= 13) {
												setting();
											}
											resetCase2();
											stateCondition = 2;
										} else {
											followBall(0);
										}
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									confirmsBall++;
								}
							}
						} else {
							if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, 1, semeh, 232, robotStatus); }
							//Walk(0.0, 0.0, 0.0);

							if (modeKick == 3) { //tendangSamping
								posPan	= -1.6;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							} else if (modeKick == 4) { //tendangSamping
								posPan	= 1.6;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							} else { //tendangDepan
								posPan	= 0.0;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							}

							panRate	= -0.05;
							tiltRate = -0.05;

							countTilt = 0;
							sumTilt = 0;
							tunda = 0;
							confirmsBall = 0;
							waitTracking = 0;
						}
					} else { //pertama
						setWaktu();
						tunggu++;
					}
				break;

				case -20: // serang lurus -> dribble
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						}
						tunggu++;
						delay = 0;
                                        } else {
						trackBall();
						tunggu = 0;

						if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
							//if (headTilt >= (cSekarang - 0.2)) {
								if (delay > 5) {
									cekWaktu(10);
									if(timer) {
										resetCase4();
										stateCondition = 4;
									} else {
										dribble(sudutTengah, 0.15);
										//followBall(0);
									}
								} else {
									setWaktu();
									delay++;
								}
							//} else {
							//	if (headTilt >= -1.0) {
							//		ballPositioning(0.0, cSekarang, 0.12);
							//	} else {
							//		followBall(0);
							//	}
							//}
						} else {
							if (headTilt >= cAktif && headPan >= -0.8 && headPan <= 0.8) {
								Imu(sudutTengah, cSekarang);
							} else {
								robotDirection = false;
								followBall(0);
							}
						}
					}
				break;

				case -10: // serang lurus -> eksekusi tendang
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							rotateKickOff(0.0, tendangJauh);
							//rotateKickOffImu(0, tendangDekat);
							//kick(tendangOper);
						}
					}
				break;

				case 10: // serang ke kanan
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(30, tendangDekat);
							} else {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(25, tendangJauh);
							}
						}
					}
				break;

				case 20: // serang ke kiri
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-30, tendangDekat);
							} else {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-25, tendangJauh);
							}
						}
					}
				break;

				case 30: // serang ke kanan solo
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							stateCondition = 90;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(30, tendangDekat);
							} else {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(25, tendangJauh);
							}
						}
					}
				break;

				case 40: // serang ke kiri solo
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							stateCondition = 100;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-30, tendangDekat);
							} else {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-25, tendangJauh);
							}
						}
					}
				break;

				case 50: // pickup 7-8
					countInitPos = 1; //lock initial pos
					cekWaktu(30);
					if (second > 5) { //5
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								Walk(0.0, 0.0, 0.0);
								resetCase1();
								stateCondition = 1;
							} else {
								if (second <= 15) {
									if (second < 11) {
										tiltSearchBall(0.0);
									} else {
										panSearchBall(-1.4);
									}
									Walk(kejar, 0.0, 0.0);
								} else {
									threeSearchBall();
									jalanDirection(kejar, 0.0, 0); //XYA
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
									if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
									//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
										resetCase2();
										stateCondition = 2;
									} else {
										followBall(0);
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 51: // pickup 0-4
					countInitPos = 1;  //lock initial pos
					if (strategy.strategyNumber == 0) {
						if (angle > 0) {
							arahRotate = -0;
						} else {
							arahRotate = 0;
						}
						searchTime = 18; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 1) {
						if (angle > 0) {
							arahRotate = -20;
						} else {
							arahRotate = 20;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 2) {
						if (angle > 0) {
							arahRotate = -40;
						} else {
							arahRotate = 40;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 3) {
						if (angle > 0) {
							arahRotate = -65;
						} else {
							arahRotate = 65;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 4) {
						if (angle > 0) {
							arahRotate = -90;
						} else {
							arahRotate = 90;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					}

					cekWaktu(searchTime);
					if (second > 5) { //5
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								Walk(0.0, 0.0, 0.0);
								resetCase1();
								stateCondition = 1;
							} else {
								threeSearchBall();
								if (second < 11) {
									saveSudutImu();
									Walk(kejar, 0.0, 0);
								} else if (second >= 11 && second <= 17){
									jalanDirection(kejar, 0.0, saveAngle + arahRotate); //XYA
								} else {
									Walk(kejar, 0.0, 0.0);
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
									if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
									//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
										resetCase2();
										stateCondition = 2;
									} else {
										followBall(0);
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 52: // pickup 13-14
					countInitPos = 1; //lock initial pos
					cekWaktu(30);
					if (second > 5) { //5
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								Walk(0.0, 0.0, 0.0);
								resetCase1();
								stateCondition = 1;
							} else {
								if (second <= 15) {
									if (second < 11) {
										tiltSearchBall(0.0);
									} else {
										panSearchBall(-1.4);
									}
									Walk(kejar, 0.0, 0.0);
								} else {
									threeSearchBall();
									jalanDirection(kejar, 0.0, 0); //XYA
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								//if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
									if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
									//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
										resetCase2();
										stateCondition = 2;
									} else {
										followBall(0);
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 53: // pickup 9-12
					countInitPos = 1;  //lock initial pos
					if (strategy.strategyNumber == 9) {
						if (angle > 0) {
							arahRotate = -20;
						} else {
							arahRotate = 20;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 10) {
						if (angle > 0) {
							arahRotate = -40;
						} else {
							arahRotate = 40;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 11) {
						if (angle > 0) {
							arahRotate = -65;
						} else {
							arahRotate = 65;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					} else if (strategy.strategyNumber == 12) {
						if (angle > 0) {
							arahRotate = -90;
						} else {
							arahRotate = 90;
						}
						searchTime = 30; // waktu searchBall sambil jalan lurus
					}

					cekWaktu(searchTime);
					if (second > 5) { //5
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								Walk(0.0, 0.0, 0.0);
								resetCase1();
								stateCondition = 1;
							} else {
								threeSearchBall();
								if (second < 11) {
									saveSudutImu();
									Walk(kejar, 0.0, 0);
								} else if (second >= 11 && second <= 17){
									jalanDirection(kejar, 0.0, saveAngle + arahRotate); //XYA
								} else {
									Walk(kejar, 0.0, 0.0);
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								//if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
									if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
									//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
										resetCase2();
										stateCondition = 2;
									} else {
										followBall(0);
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 90: // pickup ke sudut kiri
					countInitPos = 1;  //lock initial pos
					cekWaktu(30);

					if (second > 4) {
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								resetCase1();
								stateCondition = 1;
							} else {
								if (second <= 30) {						
									if (second < 20) {		
										tiltSearchBall(0.0);
									} else {						
										panSearchBall(-1.4);
									}
									Walk(kejar, 0.0, 0.0);
								} else {							
									threeSearchBall();
									jalanDirection(kejar, 0.0, 0); //XYA
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								//if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									if (useImu) {
										if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
											//if (headTilt >= (cSekarang - 0.2)) { //-1.2 //-0.7
												Walk(0.0, 0.0, 0.0);
												resetCase4();
												stateCondition = 4;
											//} else {
											//	ballPositioning(0.0, cSekarang, 0.12);
											//}
										} else {
											if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
												Imu(sudutKiri, cSekarang);
											} else {
												robotDirection = false;
												followBall(0);
											}
										}
									} else {
										if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
											Walk(0.0, 0.0, 0.0);
											resetCase4();
											stateCondition = 4;
										} else {
											followBall(0);
										}
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 100: // pickup ke sudut kanan
					countInitPos = 1;  //lock initial pos
					cekWaktu(30);

					if (second > 4) {
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								resetCase1();
								stateCondition = 1;
							} else {
								if (second <= 30) {						
									if (second < 20) {		
										tiltSearchBall(0.0);
									} else {						
										panSearchBall(-1.4);
									}
									Walk(kejar, 0.0, 0.0);
								} else {							
									threeSearchBall();
									jalanDirection(kejar, 0.0, 0); //XYA
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								//if (useCoordination) { backToCoordinations(); }					

								if (delay > 30) {
									if (useImu) {
										if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
											//if (headTilt >= (cSekarang - 0.2)) { //-1.2 //-0.7
												Walk(0.0, 0.0, 0.0);
												resetCase4();
												stateCondition = 4;
											//} else {
											//	ballPositioning(0.0, cSekarang, 0.12);
											//}
										} else {
											if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
												Imu(sudutKanan, cSekarang);
											} else {
												robotDirection = false;
												followBall(0);
											}
										}
									} else {
										if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
											Walk(0.0, 0.0, 0.0);
											resetCase4();
											stateCondition = 4;
										} else {
											followBall(0);
										}
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 130: // defense didepan
					cekWaktu(10);
					if (second < 3) {
						ball_panKP	= 0.05;
						ball_panKD	= 0.0000755;
						ball_tiltKP	= 0.05;
						ball_tiltKD	= 0.0000755;
					}

					if (timer) {
						motion("9");

						loadConfig(); //robot parameters
						resetCase2();
						stateCondition = 2;
					} else {
						// mode 1 -> tanpa menentukan jarak dengan bola
						/*if (ballLost(30)) {
							ulang	= true;
							oke	= false;

							motion("0");
							searchBallRectang(-1.4, -1.6, -0.8, 1.6);
							//panSearchBall(-1.2);
							//tiltSearchBall(0.0);
						} else {
							trackBall();
							 if (tunggu >= 60) {
								hitungGerakBola();
								if (kondisiBola == 0) {
									motion("0");
								} else {
									motion("9");

									resetCase2();
									stateCondition = 2;
								}
							} else {
								motion("0");
								tunggu++;
							}
						}*/

						// mode 2 -> menentukan jarak dengan bola
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							dapat	= false;
							motion("9");
							//Walk(0.0, 0.0, 0.0);
							//threeSearchBall();
							if ( countDef > 5 ) {
								normalSearchBall();
							} else {
								resetCase1();
								countDef++;
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(30)) {
								tracked = false;
							} else {
								trackBall();

								if (headTilt >= batasTilt && (headPan <= -0.4 || headPan >= 0.4)) {
									//if (bodyTrue == 1) {
									//	dapat = true;
									//} else {
										bodyTrackingBall(10);
									//}
								} else if (headTilt >= batasTilt && headPan >= -0.4 && headPan <= 0.4) {
									dapat = true;
								} else if (headTilt < batasTilt && headPan >= -0.4 && headPan <= 0.4) {
									followBall(0);
								}

								if (dapat) {
									if (tunggu >= 100) {
										hitungGerakBola();
										if (kondisiBola == 0) {
											motion("0");
										} else {
											motion("9");

											loadConfig(); //robot parameters
											resetCase2();
											stateCondition = 2;
										}
									} else {
										motion("0");
										tunggu++;
									}
								} else {
									motion("9");
								}
							}
						}
					}
				break;

				case 140: // defense disamping
				// disamping >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
					cekWaktu(10);
					if (second < 3) {
						ball_panKP	= 0.05;
						ball_panKD	= 0.0000755;
						ball_tiltKP	= 0.05;
						ball_tiltKD	= 0.0000755;
					}

					if (timer) {
						motion("9");

						loadConfig(); //robot parameters
						resetCase2();
						stateCondition = 2;
					} else {
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							dapat	= false;

							motion("9");
							//Walk(0.0, 0.0, 0.0);
							//threeSearchBall();
							if ( countDef > 5 ) {
								normalSearchBall();
							} else {
								resetCase1();
								countDef++;
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(30)) {
								tracked = false;
							} else {
								trackBall();

								if (headTilt >= batasTilt && (headPan <= -0.4 || headPan >= 0.4)) {
									//if (bodyTrue == 1) {
									//	dapat = true;
									//} else {
										bodyTrackingBall(10);
									//}
								} else if (headTilt >= batasTilt && headPan >= -0.4 && headPan <= 0.4) {
									dapat = true;
								} else if (headTilt < batasTilt && headPan >= -0.4 && headPan <= 0.4) {
									followBall(0);
								}

								if (dapat) {
									if (tunggu >= 100) {
										hitungGerakBola();
										if (kondisiBola == 0) {
											motion("0");
										} else {
											motion("9");

											loadConfig(); //robot parameters
											resetCase2();
											stateCondition = 2;
										}
									} else {
										motion("0");
										tunggu++;
									}
								} else {
									motion("9");
								}
							}
						}
					}

				// dibelakang >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
					/*motion("9");

					cekWaktu(7);
					if (timer) {
						resetCase2();
						stateCondition = 2;
					} else {
						if (ballLost(20)) {
							Walk(0.0, 0.0, 0.0);
							panSearchBall(-1.4);
						} else {
							trackBall();
							bodyTrackingBall(10);
						}
					}*/
				break;

				case 150: // setting receler
				break;

				case 200: // heandle error Imu
					if (strategy.bearingValue == 0) {
						zeroState = true;

						motion("0");
						headMove(batasKanan, batasAtas);
					} else {
						zeroState = false;

						resetCase1();
						stateCondition = 1;
					}
				break;

				case 1000:
					//normalSearchBall();
					motion("9");
					Walk(0.05, 0.0, 0.0);
					if (goalLost(20)) {
						//searchBallRectang2(-1.5, -1.6, -0.8, 1.6);
						tiltSearchBall(0.0);
						//panSearchBall(-0.9);
						//headMove(0.0, -1.8);
					} else {
						trackGoal();
						updateCoordinatFromVision();
					}
					//kalkulasiJarakGawang(receler.Goal_LD, receler.Goal_RD); //jarak menjadi waktu
				break;

				case 1001:	//untuk ambil data odometry per langkahnya
					//headMove(0.0,-1.5);
					getServPos();
					
					if (odometry.walkTot >= 20) {
						motion("0");
						//if ( varCount > 5 ) {
						//	cekWaktu(3);
						//	if (timer) {
						//		motion("7");
						//	}
						//} else {
						//	setWaktu();
						//	varCount++;
						//}
					} else {
						motion("9");
						jalanDirection(0.08,0.0,0);
					}
					
					/*
					odometry.bacaKondisi(odometry.RobotWalk_X, odometry.RobotWalk_Y);
					if (varCount > 200) {
						motion("0");
					} else {
						Walk(0.0,-0.03,0.0);
						motion("9");
						printf("  Lleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
						printf("  Rleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),\n", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
						varCount++;
					}
					*/
				break;

				case 1002:	//Move dengan koordinat lokalisasi
					if (varCount == 0) {
						robotPos_X = -350;
						robotPos_Y = 0;
						varCount = 1; 
					}
					motion("0");
					/*
					motion("9");
					headMove(0.0,-1.5);
					moveLokalisasi(0, 0, 0);
					if (donePosition) {
						motion("0");
					}
					*/
				break;

				default:
				break;
			}
		}
	}
	system("killall -9 screen;");
	return 0;
}
