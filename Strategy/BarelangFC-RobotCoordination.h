#ifndef BARELANGFC_ROBOTCOORDINATION_H_
#define BARELANGFC_ROBOTCOORDINATION_H_
#include "BarelangFC-RobotCoordinationData.h"

namespace BarelangFC {
	class RobotCoordination {
		private:
		public:
			int RobotID;
			//int robot1State, robot1Id, robot1FBall, robot1DBall, robot1Status;
			//int robot2State, robot2Id, robot2FBall, robot2DBall, robot2Status;
			//int robot3State, robot3Id, robot3FBall, robot3DBall, robot3Status;
			//int robot4State, robot4Id, robot4FBall, robot4DBall, robot4Status;
			//int robot5State, robot5Id, robot5FBall, robot5DBall, robot5Status;
			int robot1Id, robot1Status, robot1State, robot1GridPosition, robot1FBall, robot1DBall, robot1GridBall, robot1BackIn;
			int robot2Id, robot2Status, robot2State, robot2GridPosition, robot2FBall, robot2DBall, robot2GridBall, robot2BackIn;
			int robot3Id, robot3Status, robot3State, robot3GridPosition, robot3FBall, robot3DBall, robot3GridBall, robot3BackIn;
			int robot4Id, robot4Status, robot4State, robot4GridPosition, robot4FBall, robot4DBall, robot4GridBall, robot4BackIn;
			int robot5Id, robot5Status, robot5State, robot5GridPosition, robot5FBall, robot5DBall, robot5GridBall, robot5BackIn;

			int ROBOT_COORDINATION_PORT_OUT;

			bool refresh;

			RobotCoordination(){};
			BarelangFCCoordinationData coordinationData;

			int dataRecieve(uint8 *data, int size);

			void initCommunicationIn();
			void initCommunicationIn1();
			void initCommunicationIn2();
			void initCommunicationIn3();
			void initCommunicationIn4();
			void initCommunicationIn5();

			void readRobotCoordinationData();
			void readRobotCoordinationData1();
			void readRobotCoordinationData2();
			void readRobotCoordinationData3();
			void readRobotCoordinationData4();
			void readRobotCoordinationData5();

			void initCommunicationOut();
			void exitCommunication();

			void sendRobotCoordinationData(int rNumber, int rStatus, int sNumber, int gridPos, int fBall, int dBall, int gridBall, int backIn);

	};
}
#endif
