#ifndef BARELANGFC_ODOMETRY_H_
#define BARELANGFC_ODOMETRY_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>


namespace BarelangFC {
	class BarelangOdometry {
		private :
			#define	PI	3.1415926535897932384626433832795
			////////////// >>> GLOBAL COUNTING
			int countDiam = 0,
			fixDiam = 0,
			counterX = 0,
			counterY = 0,
			cdtTot = 0,
			////////////// >>> COUNTING DITEMPAT RIGHT FOOT
			cdtKanan = 0,
			outKanan = 0,
			indKanan = 0,
			////////////// >>> COUNTING DITEMPAT LEFT FOOT
			cdtKiri = 0,
			outKiri = 0,
			indKiri = 0,
			////////////// >>> COUNTING MAJU LEFT FOOT (X+)
			cmvKiri = 0,
			outKiriMaj = 0,
			inmKiri = 0,
			////////////// >>> COUNTING MAJU RIGHT FOOT (X+)
			cmvKanan = 0,
			outKananMaj = 0,
			inmKanan = 0,
			////////////// >>> COUNTING MUNDUR LEFT FOOT (X-)
			cbkKiri = 0,
			outKiriMun = 0,
			inmKiriMun = 0,
			////////////// >>> COUNTING MUNDUR RIGHT FOOT (X-)
			cbkKanan = 0,
			outKananMun = 0,
			inmKananMun = 0,
			////////////// >>> COUNTING SAMPING LEFT FOOT (Y+)
			outKiriSam = 0,
			indKiriSam = 0,
			////////////// >>> COUNTING SAMPING RIGHT FOOT (Y-)
			outKananSam = 0,
			indKananSam = 0,
			////////////// >>> COUNTING MUTER  LEFT FOOT (A+)
			cmtKiri = 0,
			outKiriMut = 0,
			indKiriMut = 0,
			////////////// >>> COUNTING MUTER  RIGHT FOOT (A-)
			cmtKanan = 0,
			outKananMut = 0,
			errorWalk = 0,
			indKananMut = 0;
			bool 	diam 		= false,
				ditempat 	= false,
				maju 		= false,
				mundur 		= false,
				sampingKiri 	= false,
				sampingKanan 	= false,
				muterKiri 	= false,
				muterKanan 	= false,
				useMapping 	= true;
			double coorXx, coorXy, coorYy, coorYx, sX, sY;
		public:
			bool robotBergerak;

			int walkTot, walkTotMun, csmKanan, csmKiri, RobotAngle, RobotRotate;

			double RobotPos_X, RobotPos_Y, RobotWalk_X, RobotWalk_Y, RobotWalk_A,
				R_leg1, R_leg2, R_leg3, R_leg4, R_leg5, R_leg6,							//sudut servo kaki kanan
				L_leg1, L_leg2, L_leg3, L_leg4, L_leg5, L_leg6,							//sudut servo kaki kiri
				koorXenemy, koorYenemy , koorRenemy, ArchSinEnemy,									//parameter gawang enemy
				koorXpenaltyEnemy, koorYpenaltyEnemy, koorRpenaltyEnemy, ArchSinPenEnemy,			//parameter titik penalti enemy
				koorXt, koorYt, koorRt, ArchSinTeng,								//parameter titik tengah
				koorXteam, koorYteam, koorRteam, ArchSinTeam, 							//parameter gawang team
				koorXpenaltyTeam, koorYpenaltyTeam, koorRpenaltyTeam, ArchSinPenTeam,				//parameter titik penalti team
				deltaPos_X, deltaPos_Y, 									//koordinat odometry
				initialPos_X, initialPos_Y;									//koordinat initial awal

			void bacaKondisi(double x, double y);
			void mapping(double arukuX, double arukuY);
			void trigonoMetri();
	};
}
#endif
