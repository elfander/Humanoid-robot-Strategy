#include "BarelangFC-Odometry.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace BarelangFC;
using namespace std;
void BarelangOdometry::bacaKondisi(double marcheX, double marcheY){
	if ( RobotWalk_X == 0 && RobotWalk_Y == 0 && RobotWalk_A == 0){
		if( L_leg3 == -46.34 && L_leg4 == 55.03 && R_leg3 == -46.34 && R_leg4 == 55.03) {
			countDiam += 1;
			if(countDiam > 10 && fixDiam == 0){
				//printf(" \nRobot Diam\n");
				robotBergerak = false;
				diam = true;
				ditempat = false;
				fixDiam += 1;
			}
		}else{
			countDiam = 0;
			fixDiam = 0;
			robotBergerak = true;
			diam = false;
			ditempat = true;
			/*
			if( R_leg3 >= -65 && R_leg3 <= -50 && R_leg4 >= 70 && R_leg4 <= 85 && R_leg5 >= -50 && R_leg5 <= -40 && indKanan == 0 ){//
				//printf("Langkah Kanan Ditempat = %d\n", cdtKanan);
				if(useMapping){
					mapping((marcheX + errorWalk), marcheY);
				}
				cdtKanan += 1;
				cdtTot += 1;
				outKanan = 0;
				indKanan = 1;
			}
			else if( L_leg3 >= -65 && L_leg3 <= -50 && L_leg4 >= 70 && L_leg4 <= 85 && L_leg5 >= -50 && L_leg5 <= -40 && indKiri == 0 ){//
				//printf("Langkah Kiri Ditempat = %d\n", cdtKiri);
				if(useMapping){
					mapping((marcheX + errorWalk), marcheY);
				}
				cdtKiri += 1;
				cdtTot += 1;
				outKiri = 0;
				indKiri = 1;
			}else{
				if( R_leg3 <= -30 && R_leg3 >= -45 && R_leg4 >= 50 && R_leg4 <= 65 && outKanan == 0){//
					counterX = 0;
					indKanan = 0;
					outKanan = 1;
				}else if( L_leg3 <= -30 && L_leg3 >= -45 && L_leg4 >= 50 && L_leg4 <= 65 && outKiri == 0){//
					counterX = 0;
					indKiri = 0;
					outKiri = 1;
				}
			}
			*/
		}
	}else{
		if (RobotWalk_X != 0){
			if( RobotWalk_X > 0.00){	//Bergerak Maju
				if( L_leg3 == -46.34 && L_leg4 == 55.03 && R_leg3 == -46.34 && R_leg4 == 55.03) {
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						robotBergerak = false;
						diam = true;
						maju = false;
						fixDiam +=1;
					}
				}else{
					countDiam = 0;
					fixDiam = 0;
					robotBergerak = true;
					diam = false;
					maju = true;
					if( L_leg3 >= -80  && L_leg4 >= 85  &&  L_leg5 <= -40 && inmKiri == 0){//
						//printf("Langkah Kiri Bergerak = %d\n", cmvKiri);
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						cmvKiri +=1;
						walkTot +=1;
						outKiriMaj = 0;
						walkTotMun -= 1 ;
						inmKiri = 1;
					}else if( R_leg3 >= -80  && R_leg4 >= 85 && R_leg5 <= -40 && inmKanan == 0){//
						//printf("Langkah Kanan Bergerak = %d\n", cmvKanan);
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						cmvKanan +=1;
						walkTot +=1;
						outKananMaj =  0;
						walkTotMun -= 1 ;
						inmKanan = 1;
					}else{
						if ( L_leg3 <= -30 && L_leg3 >= -45 && L_leg4 >= 50 && L_leg4 <= 65 && outKiriMaj == 0 ){//
							counterX = 0;
							inmKiri = 0;
							outKiriMaj = 1;
						} else if( R_leg3 <= -30 && R_leg3 >= -45 && R_leg4 >= 50 && R_leg4 <= 65 && outKananMaj == 0){//
							counterX = 0;
							inmKanan = 0;
							outKananMaj = 1;
						}
					}
				}
			}else if( RobotWalk_X < 0.00 ){	//Mundur
				if( L_leg3 == -46.34 && L_leg4 == 55.03 && R_leg3 == -46.34 && R_leg4 == 55.03) {
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						robotBergerak = false;
						diam = true;
						mundur = false;
						fixDiam+=1;
					}
				}else{
					countDiam = 0;
					fixDiam = 0;
					robotBergerak = true;
					diam = false;
					mundur = true;
					if( R_leg3 >= -75 && R_leg3 <= -60 && R_leg4 >= 100 && R_leg4 <= 120 && R_leg5 <= -50 && R_leg5 >=-65 && inmKananMun == 0 ){//
						//printf("Langkah Kanan Mundur = %d\n", cbkKanan);
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						cbkKanan += 1;
						walkTotMun += 1;
						outKananMun = 0;
						walkTot -= 1 ;
						inmKananMun = 1;
					}else if ( L_leg3 >= -75 && L_leg3 <= -60 && L_leg4 >= 100 && L_leg4 <= 120 && L_leg5 <= -50 && L_leg5 >=-65 &&inmKiriMun == 0 ){//
						//printf("Langkah Kiri Mundur = %d\n", cbkKiri);
						cbkKiri += 1;
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						walkTotMun += 1;
						outKiriMun = 0;
						walkTot -= 1 ;
						inmKiriMun = 1;
					}else{
						if (  R_leg3 <= -30 && R_leg3 >= -45 && R_leg5 <= -20 && R_leg5 >= -30 && outKananMun == 0){//
							counterX = 0;
							inmKananMun = 0;
							outKananMun = 1;
						} else if ( L_leg3 <= -30 && L_leg3 >= -45 && L_leg5 <=-20 && L_leg5 >= -30 && outKiriMun == 0){///
							counterX = 0;
							inmKiriMun = 0;
							outKiriMun = 1;
						}
					}
				}
			}
		} if(RobotWalk_Y != 0){
			if( RobotWalk_Y > 0.00){	//Samping Kiri
				if( L_leg3 == -46.34 && L_leg4 == 55.03 && R_leg3 == -46.34 && R_leg4 == 55.03) {
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						robotBergerak = false;
						diam = true;
						sampingKiri = false;
						fixDiam+=1;
					}
				} else{
					countDiam = 0;
					fixDiam = 0;
					robotBergerak = true;
					diam = false;
					sampingKiri = true;
					if ( L_leg2 >= 5 && L_leg2 <= 15 && L_leg3 <= -60 && L_leg3 >= -85 && L_leg4 >= 100 && L_leg4 <= 125 && L_leg5 <= -45 && L_leg5 >= -60 && indKiriSam == 0){//
						//printf("Langkah Kiri Samping = %d\n", csmKiri);
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						csmKiri += 1;
						outKiriSam = 0;
						csmKanan -= 1;
						indKiriSam = 1;
					} else if ( L_leg2 < 3 && L_leg2 >= -15 && L_leg3 >= -50 && L_leg3 <= -40 && L_leg4 >= 50 && L_leg4 <= 65 && L_leg5 <= -20 && L_leg5 >= -35 && outKiriSam == 0 ){//
						counterX = 0;
						indKiriSam = 0;
						outKiriSam = 1;
					}
				}
			} else if (RobotWalk_Y < 0.00){	//Samping Kanan
				if( L_leg3 == -46.34 && L_leg4 == 55.03 && R_leg3 == -46.34 && R_leg4 == 55.03) {
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						robotBergerak = false;
						diam = true;
						sampingKanan = false;
						fixDiam+=1;
					}
				} else{
					countDiam = 0;
					fixDiam = 0;
					robotBergerak = true;
					diam = false;
					sampingKanan = true;
					if ( R_leg2 < -5 && R_leg2 >= -15 && R_leg3 <= -60 && R_leg3 >= -85 && R_leg4 >= 100 && R_leg4 <= 125 && R_leg5 <= -45 && R_leg5 >= -60 && indKananSam == 0 ){//
						//printf("Langkah Kanan Samping = %d\n", csmKanan);
						if(useMapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						csmKanan += 1;
						outKananSam = 0;
						csmKiri -= 1;
						indKananSam = 1;
					} else if ( R_leg2 > -3 && R_leg2 <= 5 && R_leg3 >= -50 && R_leg3 <= -40 && R_leg4 >= 50 && R_leg4 <= 65 && R_leg5 <= -20 && R_leg5 >= -35 && outKananSam == 0 ){//
						counterX = 0;
						indKananSam = 0;
						outKananSam = 1;
					}
				}
			}

		} /*if(RobotWalk_A != 0){
			if( RobotWalk_A  > 0.00){	//Muter Kiri
				if( L_leg3 == -54.17 && R_leg3 == -54.17){
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						diam = true;
						muterKiri = false;
						fixDiam+=1;
					}
				}else{
					countDiam = 0;
					fixDiam = 0;
					diam = false;
					muterKiri = true;
					if (L_leg1 >=2 && L_leg3 <=-55 && L_leg4 >=70 && indKiriMut == 0){
						printf("Langkah Muter Kiri = %d\n",cmtKiri);
						if(usemamapping){
							mapping((marcheX + errorWalk), marcheY);
						}
						cmtKiri += 1;
						outKiriMut = 0;
						cmtKanan -= 1;
						indKiriMut = 1;
					} else if (L_leg1 <1 && L_leg3 >=-50 && L_leg4 <=50 && outKiriMut == 0){
						counterX = 0;
						indKiriMut = 0;
						outKiriMut = 1;
					}
				}
			} else if ( RobotWalk_A  < 0.00){	//Muter Kanan
				if( L_leg3 == -54.17 && R_leg3 == -54.17){
					countDiam+=1;
					if(countDiam > 10 && fixDiam == 0){
						//printf(" \nRobot Diam\n");
						diam = true;
						muterKanan = false;
						fixDiam+=1;
					}
				}else{
					countDiam = 0;
					fixDiam = 0;
					diam = false;
					muterKanan = true;
					if (R_leg1 <=-2 && R_leg3 <=-55 && R_leg4 >=80 && indKananMut == 0){
						printf("Langkah Muter Kanan = %d\n",cmtKanan);
						if(useMapping){
							mapping((marcheX + errorWalk),marcheY);
						}
						cmtKanan += 1;
						outKananMut = 0;
						cmtKiri -=1;
						indKananMut = 1;
					} else if (R_leg1 >-2 && R_leg3 >=-50 && R_leg4 <=50 && outKananMut == 0){
						counterX = 0;
						indKananMut = 0;
						outKananMut = 1;
					}
				}
			}
		}*/
	}
}
double 	RobotsetPointXGoalEnemy = 450,		//gawang enemy
	RobotsetPointYGoalEnemy = 0,		//gawang enemy
	RobotsetPointXpenaltyEnemy = 300,	//penalty enemy
	RobotsetPointYpenaltyEnemy = 0,		//penalty enemy
	RobotsetPointXt	= 0,	//titik tengah
	RobotsetPointYt	= 0,	//titik tengah
	RobotsetPointXGoalTeam = -450,	//gawang team
	RobotsetPointYGoalTeam = 0,	//gawang team
	RobotsetPointXpenaltyTeam = -300,	//penalty team
	RobotsetPointYpenaltyTeam = 0;		//penalty team
double zp	= 0;
void BarelangOdometry::trigonoMetri(){
	//lock for limit
	/*if(RobotPos_X >= RobotsetPointX){RobotPos_X = RobotsetPointX;}
	if(RobotPos_X <= -RobotsetPointX){RobotPos_X = -RobotsetPointX;}
	if(RobotPos_Y >= RobotsetPointY){RobotPos_Y = RobotsetPointY;}
	if(RobotPos_Y <= -RobotsetPointY){RobotPos_Y = -RobotsetPointY;}*/

	// acuan titik tengah gawang lawan (450,0)
	koorXenemy = RobotsetPointXGoalEnemy - RobotPos_X;
	koorYenemy = RobotsetPointYGoalEnemy - RobotPos_Y;
	koorRenemy = sqrt((koorXenemy*koorXenemy)+(RobotPos_Y*RobotPos_Y));
	//ArchSinEnemy = zp - asin(RobotPos_Y/koorRenemy)*(180/PI);
	if (RobotPos_X >= 450) {		//Target ada dibelakang
		if (RobotPos_Y >= 0) {	//saat ini sebelah kanan
			ArchSinEnemy = -180 + asin(RobotPos_Y/koorRenemy)*(180/PI);
		} else if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinEnemy = 180 + asin(RobotPos_Y/koorRenemy)*(180/PI);
		}
	} else if (RobotPos_X < 450) {	//Target ada didepan
		if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinEnemy = 0 - asin(RobotPos_Y/koorRenemy)*(180/PI);
		} else if (RobotPos_Y >= 0) {	//saat ini sebalah kanan
			ArchSinEnemy = 0 - asin(RobotPos_Y/koorRenemy)*(180/PI);
		}
	}	
        //printf("ArchSinEnemy = %.f\n",ArchSinEnemy);
        
	// acuan titik tengah lapangan (0,0)
	koorXt = RobotsetPointXt - RobotPos_X;
	koorYt = RobotsetPointYt - RobotPos_Y;
	koorRt = sqrt((koorXt*koorXt)+(RobotPos_Y*RobotPos_Y));
	//ArchSinTeng = zp - asin(RobotPos_Y/koorRt)*(180/PI);
	if (RobotPos_X >= 0) {		//Target ada dibelakang
		if (RobotPos_Y >= 0) {	//saat ini sebelah kanan
			ArchSinTeng = -180 + asin(RobotPos_Y/koorRt)*(180/PI);
		} else if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinTeng = 180 + asin(RobotPos_Y/koorRt)*(180/PI);
		}
	} else if (RobotPos_X < 0) {	//Target ada didepan
		if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinTeng = 0 - asin(RobotPos_Y/koorRt)*(180/PI);
		} else if (RobotPos_Y >= 0) {	//saat ini sebalah kanan
			ArchSinTeng = 0 - asin(RobotPos_Y/koorRt)*(180/PI);
		}
	}		
        //printf("ArchSinTeng = %.f\n",ArchSinTeng);
        
	// acuan titik penalti lawan (300,0)
	koorXpenaltyEnemy = RobotsetPointXpenaltyEnemy - RobotPos_X;
	koorYpenaltyEnemy = RobotsetPointYpenaltyEnemy - RobotPos_Y;
	koorRpenaltyEnemy = sqrt((koorXpenaltyEnemy*koorXpenaltyEnemy)+(RobotPos_Y*RobotPos_Y));
	//ArchSinPenEnemy = zp - asin(RobotPos_Y/koorRpenaltyEnemy)*(180/PI);
	if (RobotPos_X >= 300) {		//Target ada dibelakang
		if (RobotPos_Y >= 0) {	//saat ini sebelah kanan
			ArchSinPenEnemy = -180 + asin(RobotPos_Y/koorRpenaltyEnemy)*(180/PI);
		} else if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinPenEnemy = 180 + asin(RobotPos_Y/koorRpenaltyEnemy)*(180/PI);
		}
	} else if (RobotPos_X < 300) {	//Target ada didepan
		if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinPenEnemy = 0 - asin(RobotPos_Y/koorRpenaltyEnemy)*(180/PI);
		} else if (RobotPos_Y >= 0) {	//saat ini sebalah kanan
			ArchSinPenEnemy = 0 - asin(RobotPos_Y/koorRpenaltyEnemy)*(180/PI);
		}
	}	
        //printf("ArchSinPenEnemy = %.f\n",ArchSinPenEnemy);	

	// acuan titik penalti team (-300,0)
	koorXpenaltyTeam = RobotsetPointXpenaltyTeam - RobotPos_X;
	koorYpenaltyTeam = RobotsetPointYpenaltyTeam - RobotPos_Y;
	koorRpenaltyTeam = sqrt((koorXpenaltyTeam*koorXpenaltyTeam)+(RobotPos_Y*RobotPos_Y));
	//ArchSinPenTeam = zp - asin(RobotPos_Y/koorRpenaltyTeam)*(180/PI);
	if (RobotPos_X >= -300) {		//Target ada dibelakang
		if (RobotPos_Y >= 0) {	//saat ini sebelah kanan
			ArchSinPenTeam = -180 + asin(RobotPos_Y/koorRpenaltyTeam)*(180/PI);
		} else if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinPenTeam = 180 + asin(RobotPos_Y/koorRpenaltyTeam)*(180/PI);
		}
	} else if (RobotPos_X < -300) {	//Target ada didepan
		if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinPenTeam = 0 - asin(RobotPos_Y/koorRpenaltyTeam)*(180/PI);
		} else if (RobotPos_Y >= 0) {	//saat ini sebalah kanan
			ArchSinPenTeam = 0 - asin(RobotPos_Y/koorRpenaltyTeam)*(180/PI);
		}
	}	
        //printf("ArchSinPenTeam = %.f\n",ArchSinPenTeam);

	// acuan titik tengah gawang team (-450,0)
	koorXteam = RobotsetPointXGoalTeam - RobotPos_X;
	koorYteam = RobotsetPointYGoalTeam - RobotPos_Y;
	koorRteam = sqrt((koorXteam*koorXteam)+(RobotPos_Y*RobotPos_Y));
	//ArchSinTeam = zp - asin(RobotPos_Y/koorRteam)*(180/PI);
	if (RobotPos_X >= -450) {		//Target ada dibelakang
		if (RobotPos_Y >= 0) {	//saat ini sebelah kanan
			ArchSinTeam = -180 + asin(RobotPos_Y/koorRteam)*(180/PI);
		} else if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinTeam = 180 + asin(RobotPos_Y/koorRteam)*(180/PI);
		}
	} else if (RobotPos_X < -450) {	//Target ada didepan
		if (RobotPos_Y < 0) {	//saat ini sebelah kiri
			ArchSinTeam = 0 - asin(RobotPos_Y/koorRteam)*(180/PI);
		} else if (RobotPos_Y >= 0) {	//saat ini sebalah kanan
			ArchSinTeam = 0 - asin(RobotPos_Y/koorRteam)*(180/PI);
		}
	}	
        //printf("ArchSinTeam = %.f\n",ArchSinTeam);
}

void BarelangOdometry::mapping(double arukuX, double arukuY){
	//value use minimal / 10
	if (arukuX > 0){
		if ( arukuX > 0 && arukuX <= 0.03){
			sX = 2.4 * arukuX / 0.03; //2.40	//2.05
		} else if (arukuX > 0.03 && arukuX <= 0.04){
			sX = 3.6 * arukuX / 0.04; //3.15	//2.8688
		} else if (arukuX > 0.04 && arukuX <= 0.05){
			sX = 5.2 * arukuX / 0.05;	//4.15	//3.818
		} else if ( arukuX > 0.05 && arukuX <= 0.06){
			sX = 5.714285714285714 * arukuX / 0.06;	//5	//4.59
		} else if ( arukuX > 0.06 && arukuX <= 0.07){
			sX = 6.904761904761905 * arukuX / 0.07;
		} else if ( arukuX > 0.07 && arukuX <= 0.08){
			sX = 7.619047619047619 * arukuX / 0.08;
		}
	} else if (arukuX < 0 ){
		if (arukuX == -0.01){
			sX = -0.8;
		} else if ( arukuX == -0.02){
			sX = -1.2;
		} else if (arukuX == -0.03){
			sX = -1.8;
		} else {
			sX = -1.8* arukuX / -0.03;
		}
	} else if (arukuX == 0){
		sX = 0;
	}
	
	if (arukuY > 0 ){
		//printf("POSITIF\n"); //LEFT
		if (arukuY > 0 && arukuY <= 0.01){
			sY = 2 * arukuY / 0.01;	//2.5
		} else if (arukuY > 0.01 && arukuY <= 0.02){
			sY = 4.2 * arukuY / 0.02;		//4
		} else if ( arukuY > 0.02 && arukuY <= 0.03){
			sY = 6.3 * arukuY / 0.03;		//5	//6.54
		} else {		
			sY = 6.3 * arukuY /0.03;	//4.75
		}
	} else if (arukuY < 0 ){
		//printf("NEGATIF\n");
		/*if (arukuY == -0.01){
			sY = -1.333;
		} else if (arukuY == -0.02){
			sY = -2.667;
		} else if (arukuY == -0.03){
			sY = -4;
		} else {
			sY = -1.333*arukuY / -0.01;
		}*/

		if (arukuY < 0 && arukuY >= -0.01){
			sY = -1.9 * arukuY / -0.01;	//2.5
		} else if (arukuY < -0.01 && arukuY >= -0.02){
			sY = -4.1 * arukuY / -0.02;	//-4
		} else if ( arukuY < -0.02 && arukuY >= -0.03){
			sY = -6.3 * arukuY / -0.03;	//-5
		} else {
			sY = -6.3 * arukuY / -0.03;	//-4.75
		}
	} else if (arukuY == 0){
		sY = 0;
	}
	//bagian X
	coorXx = (sX * (cos((abs(RobotAngle)) * PI / 180)));
	//coorXx = sX * cos(abs(RobotAngle)*PI/180);
	//coorXy = sX * sin(RobotAngle*PI/180);
	coorXy = (sX * (sin(RobotAngle * PI / 180)));
	//bagian Y
	if (RobotAngle <= 180 && RobotAngle >= -90){
		coorYx = (sY * (cos((abs(RobotAngle - 90)) * PI / 180)));
		//coorYx = sY * cos(angleYm*PI/180);
		coorYy = sY * sin((RobotAngle-90)*PI/180);
	} else {
		coorYx = (sY * (cos((abs(RobotAngle + 270)) * PI / 180)));
		//coorYx = sY * cos(angleYm*PI/180);
		coorYy = (sY * (sin((RobotAngle + 270) * PI / 180)));
	} 

	if (counterX == 0 /*|| counterY == 0*/){
		deltaPos_X = deltaPos_X + (0.8*coorXx) + (0.8*coorYx);	//komen Yx
		deltaPos_Y = deltaPos_Y + (0.9*coorXy) + coorYy;	//Xy = x1.1
		//trigonoMetri();
		counterX = 1;
	}

}
