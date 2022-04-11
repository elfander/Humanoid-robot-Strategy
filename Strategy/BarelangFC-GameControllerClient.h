#ifndef BARELANGFC_GAMECONTROLLERCLIENT_H_
#define BARELANGFC_GAMECONTROLLERCLIENT_H_

#include "RoboCupGameControlData.h"

namespace BarelangFC {
	class GameControllerClient{
		private:
			int len, flags, broadcast, sock_fd, nGameControlData;
			int sock_rt;
			bool init_read, init_return;
			double recvTime;
			char data[4096];

		public:
			int 	State, Player, Team,
				FirstHalf,
				Version,
				PacketNumber,
				PlayerTeam,
				GameTipe,
				KickOff,
				SecondaryState,
				DropTeam,
				DropTime,
				Remaining,
				SecondaryTime,
						// ket : 1 = untuk data GameController yang kiri
						//	 2 = untuk data GameController yang kanan
				timNumber1,
				timNumber2,
				timColour1,
				timColour2,
				Score1,
				Score2,
				Penaltyshoot1,
				Penaltyshoot2,
				Singleshoot1,
				Singleshoot2,
				Coachsequence1,
				Coachsequence2,

				Penalty1,
				Penalty2,
				TimeUnpenalis1,
				TimeUnpenalis2,
				YellowCard1,
				YellowCard2,
				RedCard1,
				RedCard2;

			bool 	Penalise = true;

			const char* Server;
			RoboCupGameControlData gameControlData;
			RoboCupGameControlReturnData gameControlReturnData;
			void mexExit();
			void readGameControllerData();
			void returnGameControllerData();
	};
}
#endif
