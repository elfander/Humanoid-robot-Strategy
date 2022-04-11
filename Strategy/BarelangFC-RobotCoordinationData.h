#include "BarelangFC-RobotCoordination.h"

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

#define ROBOT_COORDINATION_PORT_IN_1		5041
#define ROBOT_COORDINATION_PORT_IN_2		5042
#define ROBOT_COORDINATION_PORT_IN_3		5043
#define ROBOT_COORDINATION_PORT_IN_4		5044
#define ROBOT_COORDINATION_PORT_IN_5		5045

//#define ROBOT_COORDINATION_PORT_OUT		4044
#define ROBOT_COORDINATION_STRUCT_HEADER	"BarelangFC"

struct BarelangFCCoordinationData {
    char header[10];
    //uint8 robotNumber;
    //uint8 foundBall;          // 0 = tidak, 1 ya
    //uint32 distanceBall;      // jarak bola yg dilihat
    //uint8 stateNumber;        // colour of the goal
    //uint8 robotStatus;
    uint8 robotNumber;          //robot id
    //uint8 robotRole;
    uint8 robotStatus;          //status play or kill => 1 : play, 0 : kill
    uint8 stateNumber[2];       //number statecondition
    uint8 robotGrid;            //number grid posisi robot
    uint8 foundBall;            //status found or not found ball => 1 : found, 0 : not found
    uint8 distanceBall[2];      //jarak bola terhadap robot
    uint8 ballGrid;             //number grid posisi bola
    uint8 backIn;               //koordinasi ketika balik untuk pickup langsung menyesuaikan
};
