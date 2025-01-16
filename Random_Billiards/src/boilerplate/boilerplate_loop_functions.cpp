#include "boilerplate_loop_functions.h"
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/entities/tag_entity.h>
#include <argos3/plugins/simulator/entities/radio_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/media/radio_medium.h>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include<cmath>
#include <array>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <string> 
#include <fstream>
#include <exception>
#include <experimental/filesystem>
#include <iostream>
#include <functional>
#include <fstream>
#include <vector>
namespace argos {
	const int Nb_robots = 8;
	const int Nb_blocks = 100;
	float Robots[Nb_robots]; // to record the total deitance each robot travelled
	double xCoordinates[Nb_blocks]; // to read X coordinate of the positions of the obstacles from the csv file (for Nb_blocks obstacles)
	double yCoordinates[Nb_blocks]; // to read Y coordinate of the positions of the obstacles from the csv file (for Nb_blocks obstacles)
	double wOrientations[Nb_blocks]; // to read W component of the quaternions (orientations) of the obstacles from the csv file (for Nb_blocks obstacles)
	double xOrientations[Nb_blocks]; // to read X component of the quaternions (orientations) of the obstacles from the csv file (for Nb_blocks obstacles)
	double yOrientations[Nb_blocks]; // to read Y component of the quaternions (orientations) of the obstacles from the csv file (for Nb_blocks obstacles)
	double zOrientations[Nb_blocks]; // to read Z component of the quaternions (orientations) of the obstacles from the csv file (for Nb_blocks obstacles)
	int stepSum = 0; // for counting the total number of messages sent in one step; not used in this project
	float RobotCollision[Nb_robots][2]; // for Nb_robots ground robots; record the current position of each robot
	int collision[Nb_robots][Nb_robots]={}; // a counter used to for collistion detection between each pair or robots (for Nb_robots robots)
    int collisionCounter[Nb_robots][Nb_robots]={}; // for Nb_robots ground robots
	int sumOfCollisions = 0; // total number of collisions
	int c3 = 0;
	float temp3 = 0.00001;
	double myX[Nb_robots] = {}; // used to keep the X coordinate of the position of the faulty robots (when they stop moving in a faut tolerance experiment)
	double myY[Nb_robots] = {}; // used to keep the Y coordinate of the position of the faulty robots (when they stop moving in a faut tolerance experiment)
	double myW_Orientation[Nb_robots]; // used to keep the W component of the quaternion of the faulty robots (when they stop moving in a faut tolerance experiment)
	double myX_Orientation[Nb_robots]; // used to keep the X component of the quaternion of the faulty robots (when they stop moving in a faut tolerance experiment)
	double myY_Orientation[Nb_robots]; // used to keep the Y component of the quaternion of the faulty robots (when they stop moving in a faut tolerance experiment)
	double myZ_Orientation[Nb_robots]; // used to keep the Z component of the quaternion of the faulty robots (when they stop moving in a faut tolerance experiment)
	int randFailure[6] = {}; //used to randomly select faulty robots in a faut tolerance experiment  
	int new_flag = 0; // used in fault tolerance
	int rand_num; // used in fault tolerance
	int counter = 0; // used in fault tolerance
	std::string IDs[6] = {}; // to record ID of faulty robots
	double robotPos[Nb_robots][2] = {5}; // used in the initialization phase for placing the robots
	int c2 = 0; // a counter for robots
   /****************************************/
   /****************************************/

	void CBoilerplateLoopFunctions::Init(TConfigurationNode& t_tree) {
		sum = 0; // total number of messages passed within the swarm
		totalDis = 0; // total distance travelled by the swarm
		temp = 0;
		srand(time(0));
		// intialize files for recording data: lines 67 to 125
		/* Get output file name from XML tree */
		GetNodeAttribute(t_tree, "output", m_strOutFile);
		std::time_t t = std::time(0);   // get time now
		std::tm* now = std::localtime(&t);
		std::cout << (now->tm_year + 1900) << '-' 
		<< (now->tm_mon + 1) << '-'
		<<  now->tm_mday << '_'
		<< now->tm_hour << ":"
		<< now->tm_min << ":"
		<< now->tm_sec
		<< "\n";
		auto tt = std::time(nullptr);
		auto tmt = *std::localtime(&tt);
		oss << std::put_time(&tmt, "%d-%m-%Y_%H-%M-%S");
		auto str = oss.str();
		str = "experiments/" + str;

		const int dir_err = mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (-1 == dir_err)
		{
			printf("Error creating directory!n");
			exit(1);
		};
		m_strOutFile3 = str + "/boilerplate.argos";
		std::ifstream  src("boilerplate.argos", std::ios::binary);
		std::ofstream  dst(m_strOutFile3.c_str(),   std::ios::binary);
		dst << src.rdbuf();

		m_strOutFile = str + "/" + m_strOutFile;
		std::cout << str << std::endl;
		LOG << "File: " << str << std::endl;
		LOG << "tree" << m_strOutFile << std::endl;
		/* Open the file for text writing */
		m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
		if(m_cOutFile.fail()) {
			THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\"");
		}

		/* Get output file name from XML tree */
		GetNodeAttribute(t_tree, "output3", m_strOutFile8);
		m_strOutFile8 = str + "/" + m_strOutFile8;
		/* Open the file for text writing */
		m_cOutFile8.open(m_strOutFile8.c_str(), std::ofstream::out | std::ofstream::trunc);
		if(m_cOutFile8.fail()) {
			THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile8 << "\"");
		}

		/* Get output file name from XML tree */
		GetNodeAttribute(t_tree, "output2", m_strOutFile2);
		m_strOutFile2 = str + "/" + m_strOutFile2;
		/* Open the file for text writing */
		m_cOutFile2.open(m_strOutFile2.c_str(), std::ofstream::out | std::ofstream::trunc);
		if(m_cOutFile2.fail()) {
			THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile2 << "\"");
		}

		m_strOutFile10 = str + "/blocks.csv";
		std::ifstream  src6("blocks.csv", std::ios::binary);
		std::ofstream  dst6(m_strOutFile10.c_str(),   std::ios::binary);	
		dst6 << src6.rdbuf();	

	}

   /****************************************/
   /****************************************/
	bool CBoilerplateLoopFunctions::IsExperimentFinished() {
		if (terminationCount==500){  // termination time step
			return true;		
		}
		return false;

	}  

   /****************************************/
   /****************************************/
	void CBoilerplateLoopFunctions::PreStep() {
		if (simcount==0){
			m_cOutFile10.open("FaultTolerance/faulty_robots.csv", std::ofstream::out | std::ofstream::trunc);
			const CVector3& cArenaSize = CSimulator::GetInstance().GetSpace().GetArenaSize();
			std::cerr << "PrePhase: ArenaSize: " << cArenaSize << std::endl;
			a = cArenaSize[0]-1; // a fraction of the length of the main field's border along x axis (i.e., length_of_the_border/4)
			b = cArenaSize[1]-1; // a fraction of the length of the main field's border along y axis (i.e., length_of_the_border/4)
			//create the mapping matrix for Arena Matrix: a 16 by 16 matrix (the cells are of equal size: 0.25 cm x 0.25 cm)
			float tempA = a/2;
			float tempB = -b/2;
			for (int i=0;i<(4*a);i++){
				for(int j=0;j<(4*b);j++){
					// the 4 following entries record the bounds of each cell (x coordinate and y coordinate ranges that bound a cell)
					enVec[i][j][0] = tempA; 
					enVec[i][j][1] = tempA-0.25f;
					enVec[i][j][2] = tempB+0.25f; 
					enVec[i][j][3] = tempB;
					enVec[i][j][4] = 0; // a counter to record the total time spent by robots in a cell
					enVec[i][j][5] = 0; // a counter to record the total number of visits in a cell			
					tempB = tempB+0.25f;	
				}
				tempA = tempA-(0.25f);
				tempB = -b/2;
			}
			for(int i=0;i<0;i++){ //for 6 failures: for(int i=0;i<6;i++){ //for 4 failures: for(int i=0;i<4;i++){ //for 2 failures: for(int i=0;i<2;i++){
				new_flag = 0;
				while(new_flag == 0){
					rand_num = rand()%8+1;
					counter = 0;
					for (int j=0;j<6;j++){
						if (rand_num != randFailure[j]){
							counter = counter + 1;
						}
					}
					if (counter == 6){
						new_flag = 1;
					}
				} 
				randFailure[i] = rand_num;
				if(rand_num == 1){
					IDs[i] = "vehicle0";
					m_cOutFile10 << 0
					<< std::endl;
				}
				else if(rand_num == 2){
					IDs[i] = "vehicle1";
					m_cOutFile10 << 1
					<< std::endl;
				}
				else if(rand_num == 3){
					IDs[i] = "vehicle2";
					m_cOutFile10 << 2 
					<< std::endl;
				}
				else if(rand_num == 4){
					IDs[i] = "vehicle3";
					m_cOutFile10 << 3 
					<< std::endl;
				}
				else if(rand_num == 5){
					IDs[i] = "vehicle4";
					m_cOutFile10 << 4
					<< std::endl;
				}
				else if(rand_num == 6){
					IDs[i] = "vehicle5";
					m_cOutFile10 << 5
					<< std::endl;
				}
				else if(rand_num == 7){
					IDs[i] = "vehicle6";
					m_cOutFile10 << 6 
					<< std::endl;
				}
				else if(rand_num == 8){
					IDs[i] = "vehicle7";
					m_cOutFile10 << 7 
					<< std::endl;
				}
			}
			m_cOutFile10.close();
			// read the blocks.csv file and based on it place the obstacles in the environment: lines  223 to 256
			int cc = 0;
			std::ifstream classFile("blocks.csv");
			std::string line;
			std::vector<std::string> row;
			std::setlocale(LC_ALL, "C");
			// read the blocks.csv file	
			while (std::getline(classFile, line,'\n')){
				std::istringstream s(line);
				std::string field;
				int mycount = 1;
				while (getline(s, field,',')){	
					if (mycount == 1){
						xCoordinates[cc] = stof(field);
					}
					if (mycount == 2){
						yCoordinates[cc] = stof(field);
					}
					if (mycount == 3){
						wOrientations[cc] = stof(field);
					}
					if (mycount == 4){
						xOrientations[cc] = stof(field);
					}
					if (mycount == 5){
						yOrientations[cc] = stof(field);
					}
					if (mycount == 6){
						zOrientations[cc] = stof(field);
					}
					row.push_back(line);
					mycount = mycount + 1;
				}
				cc = cc + 1;
			}
			CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("prototype"); // in order to create a reference to the obstacles and the robots for easier coding
			for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
				/* Create a reference for easier coding */
				CPrototypeEntity* cFB = any_cast<CPrototypeEntity*>(it->second); // create a reference to the obstacles and the robots for easier coding
				if (cFB->HasComponent("leds.led[led_0]")){ // if the entity is an obstacle (the obstacles have an LED on their top; please see the boilerplate.argos file)
					// place the obstacles in the environment based on the information read from blocks.csv
					if (simcount == 0){ // before that the experiment starts
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetX(xCoordinates[c3]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetY(yCoordinates[c3]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetW(wOrientations[c3]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetX(xOrientations[c3]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetY(yOrientations[c3]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetZ(zOrientations[c3]);
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
						c3 = c3 + 1;
					}
				
				}
				int robot_distribution;
				float randX;
				float randY;
				if (cFB->HasComponent("tags")){
					robot_distribution = 0;
					if (simcount == 0){
						while (robot_distribution == 0){
							randX = ((float)rand() / RAND_MAX) * (1.95 + 1.95) - 1.95;
							randY = ((float)rand() / RAND_MAX) * (1.95 + 1.95) - 1.95;
							int checkRand = 0;
							for(j=0;j<Nb_blocks;j++){ // when the robots are distributed the distance between a robot and any obstacle should be more than 6 cm
								temp3 = sqrt(pow((xCoordinates[j]-randX),2)+pow((yCoordinates[j]-randY),2));
								if(temp3<0.06){
									checkRand = 1;
								} 
							}
							for(j=0;j<Nb_robots;j++){ // when the robots are distributed the distance between each pair of robots should be more than 50 cm
								temp3 = sqrt(pow((robotPos[j][0]-randX),2)+pow((robotPos[j][1]-randY),2));
								if(temp3<0.5){
									checkRand = 1;
								} 
							}					
							if (checkRand == 0){
								robot_distribution = 1;
							}
						}
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetX(randX);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetY(randY);
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
						robotPos[c2][0] = randX;
						robotPos[c2][1] = randY;
						c2 = c2+1;
						/* Create a reference for easier coding */
						m_tWaypoints[cFB] = std::vector<CVector3>();
						// record the position of the robots in a vector in order to calculate the total distance travelled by them
						m_tWaypoints[cFB].push_back(cFB->GetEmbodiedEntity().GetOriginAnchor().Position);									
					}
				}
			}//for space
		}
	}
   /****************************************/
   /****************************************/

	void CBoilerplateLoopFunctions::PostStep() {
		temp = 0;
		int c=0; // a counter for aground robots
		simcount = simcount + 1; // simulation step is increased by 1
		currentPercentage = 0; // to record the current coverage completeness
		zeroCounter = 0; // to record the number of unvisited cells
		CSpace::TMapPerType& cFBMap = GetSpace().GetEntitiesByType("prototype"); // in order to create a reference to the obstacles and the robots for easier coding
		for(CSpace::TMapPerType::iterator it = cFBMap.begin(); it != cFBMap.end(); ++it) {
			/* Create a reference for easier coding */
			CPrototypeEntity* cFB = any_cast<CPrototypeEntity*>(it->second);
			// calculate and record the number of messages sent by a robot in the current step and the total number of messages sent within the swarm: lines: 330 to 338		
			if (cFB->HasControllableEntity()){
				CControllableEntity& cControllableEntity = cFB->GetControllableEntity();
				CCI_Controller& cController = cControllableEntity.GetController();
				CCI_RadiosActuator* cRadiosActuator = cController.GetActuator<CCI_RadiosActuator>("radios");
				const CCI_RadiosActuator::SInterface sInterface = cRadiosActuator->GetInterfaces()[0];
				sum = sum  + sInterface.Data.size(); // calculating the total number of messages sent within the swarm
				stepSum = stepSum + sInterface.Data.size(); // calculating the number of messages sent within the swarm in the current time step
				m_cOutFile << sInterface.Data.size() << "\t" ;
			}
			if (cFB->HasComponent("tags")){ // for each robot	
				if (c>=0){
					// to keep the faulty robot stationary from time step 500 (which is recorded as time step 500); lines: 342 to 365 
					int failureFlag = 0;
					for (int i=0;i<6;i++){
						if(simcount >= 500 && cFB->GetId() == IDs[i]){
							failureFlag = 1;
						}
					}
					if (failureFlag == 1){
						if (simcount == 500){
							myX[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
							myY[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
							myW_Orientation[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW();
							myX_Orientation[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX();
							myY_Orientation[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY();
							myZ_Orientation[c] = cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ();
						}
						
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetW(myW_Orientation[c]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetX(myX_Orientation[c]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetY(myY_Orientation[c]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.SetZ(myZ_Orientation[c]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetX(myX[c]);
						cFB->GetEmbodiedEntity().GetOriginAnchor().Position.SetY(myY[c]);
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
					}
					RobotCollision[c][0] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]; // current x coordinate of the robot in the loop 
					RobotCollision[c][1] = cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]; // current y coordinate of the robots in the loop
					
					// the following if-elseif statement (lines 370 to 421) is used to simulate random turn when a robot reaches a border
					if (cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]<-2){
						v = rand() % 2;
						if (v == 0) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, 180);
						}
						else if (v == 1) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, v1);
						} 
						
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
					}
					else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]<-2){
						v = rand() % 2;
						if (v == 0) {
							v1 = rand() % 180 ;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, v1);
						}
						else if (v == 1) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, -1*v1);
						} 
						
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
					}
					else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]>2){
						v = rand() % 2;
						if (v == 0) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, -180);
						}
						else if (v == 1) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(180,0,0, -1*v1);
						} 
						
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
					}
					else if(cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]>2){
						v = rand() % 2;
						if (v == 0) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(v1,0,0, 180);
						}
						else if (v == 1) {
							v1 = rand() % 180;
							cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation.Set(-1*v1,0,0, 180);
						} 
						
						MoveEntity(cFB->GetEmbodiedEntity(), cFB->GetEmbodiedEntity().GetOriginAnchor().Position, cFB->GetEmbodiedEntity().GetOriginAnchor().Orientation);
					}		

					// for each cell calculate the total robots spent in it and the number of visits
					for (int i=0;i<(4*a);i++){
						for(int j=0;j<(4*b);j++){
							if (cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1] <= enVec[i][j][0] && cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1] > enVec[i][j][1] && cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0] <= enVec[i][j][2] && cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0] > enVec[i][j][3]){
								enVec[i][j][4] = enVec[i][j][4] + 1;
								if ((simcount==1 && enVec[i][j][4]>0) || ((m_tWaypoints[cFB].back()[1] > enVec[i][j][0]) || (m_tWaypoints[cFB].back()[1] <= enVec[i][j][1]) || (m_tWaypoints[cFB].back()[0] > enVec[i][j][2]) || (m_tWaypoints[cFB].back()[0] <= enVec[i][j][3]))) {
									enVec[i][j][5] = enVec[i][j][5] + 1;
								}
							}
						}
					}

					// if the distance between the current position of robot i and its position in the previous step is greater than 0.1 cm, add the value to 
					// the Robot array's entery which records the total distance teravelled by that robot
					temp = pow(pow((cFB->GetEmbodiedEntity().GetOriginAnchor().Position[1]-m_tWaypoints[cFB].back()[1]),2)+pow((cFB->GetEmbodiedEntity().GetOriginAnchor().Position[0]-m_tWaypoints[cFB].back()[0]),2),1/2.0);
					if(temp > 0.001) {
						m_tWaypoints[cFB].push_back(cFB->GetEmbodiedEntity().GetOriginAnchor().Position);// record the position of the robots in a vector in order to calculate the total distance travelled by them
						Robots[c] = Robots[c]+temp; // add the value to Robot array's entery which records the total distance teravelled by that robot
						totalDis = totalDis + temp; // add the value to the total distance travelled by the swarm of ground robots
					}
					m_cOutFile << Robots[c] << "\t" ;
				}

				c=c+1;
				if (c==Nb_robots) {
					stepSum = 0; // reset the variable that count the total number of messages sent within the swarm at the current time step
				}
			} // end of if statement related to the robots (tag); 
			
		}//end of the for loop of robot space

		//computing distances between the robots and counting the collisions when the distance is less than 10 cm (robot center to robot center)
		float disGround = 1000;

		for (int l=0; l<Nb_robots; l++){
			for(int j=l+1; j<Nb_robots; j++){
				if (collision[l][j] > 0) {
					collision[l][j] = collision[l][j] - 1;
				}
				disGround = pow(pow((RobotCollision[l][0]-RobotCollision[j][0]),2)+pow((RobotCollision[l][1]-RobotCollision[j][1]),2),1/2.0);
				if (disGround < 0.1){
					if (collision[l][j] > 0) {
						collision[l][j] = collision[l][j] + 1;
					}
					else if (collision[l][j] == 0){
						collision[l][j] = 20; //setting the time counter for next collision detection 
						collisionCounter[l][j] = collisionCounter[l][j] + 1;
						sumOfCollisions = sumOfCollisions + 1;
					}
				}
			}
		}

		float max = -1;
		float min = 200000.0;
		for (int i=0;i<(4*a);i++){
			for(int j=0;j<(4*b);j++){
				if(enVec[i][j][4] == 0){
					zeroCounter = zeroCounter + 1; // counting the number of unvisited cells
				}
				if (enVec[i][j][4] > max){	
					max = enVec[i][j][4]; // record maximum time robots collectively spent in a cell
				}
				if (enVec[i][j][4] < min){	
					min = enVec[i][j][4]; // record minimum time robots collectively spent in a cell (it is 0 if there is at least one unvisited cell)
				}			      
			}		
		}
		//-------------------------------------------------------------------------------------
		//-------------------------------------------------------------------------------------
		//calculate the uniformity formula for the total time spent in cell
		int med_p1 = 0; //median for p = 1
		double min_sum_p1 = 2000000.0;
		double sumation_p1 = 0.0;
		double Median_div_visited_p1 = 100000; // a big number: for now has been set to 100000 to prevent scientific notation
		for (float c=0; c<=max; c++){
			sumation_p1 = 0.0;
			for (int i=0; i<16; i++){
				for(int j=0; j<16; j++){
					sumation_p1 = sumation_p1 + abs(enVec[i][j][4] - c);
				}
				
			}
			if (sumation_p1 < min_sum_p1) {
				min_sum_p1 = sumation_p1;
				med_p1 = c;
			}
		}
		if ((256-zeroCounter) != 0){
			Median_div_visited_p1 = min_sum_p1/(256-zeroCounter);
		} 
		double new_uniformity = std::sqrt(Median_div_visited_p1); //the output of the formula (first calculate the summation (line 502), then divide it by #visited cells, and finally calculate its second root) 
		//-------------------------------------------------------------------------------------
		//-------------------------------------------------------------------------------------
		// record the data
		currentPercentage = (256-zeroCounter)*100/256.0;
		LOG << "current Percentage: " << currentPercentage << std::endl;	
		m_cOutFile << totalDis << "\t"
		<< sum << "\t"
		<< sumOfCollisions << "\t"
		<< (256-zeroCounter)*100/256.0 << "\t" //comment if the entire arena is considered
		<< 256-zeroCounter << "\t"
		<< zeroCounter << "\t"
		<< new_uniformity << "\t"
		<< GetSpace().GetSimulationClock()
		<< std::endl;

		// any experiment of the decentralized approaches terminates when in 500 steps no new cell is found by the ground robots:
		if (zeroCounter == lastzeroCounter){ // if the number of non-visited cells of the current step is equal to that of the previous step
			terminationCount = terminationCount + 1; // increase the termination counter by 1
		}
		else { // a new cell has been visited at this step
			lastzeroCounter = zeroCounter; // record the number of non-visited cells of the current step 
			terminationCount = 0; // reset the terminationCount counter
		}

		if (terminationCount == 500){ // at the termination time step
			// record the total time spent by robots in each cell
			for (int i=0;i<(4*a);i++){
				for(int j=0;j<(4*b);j++){
					m_cOutFile2 << enVec[i][j][4] << "\t";
				}
				m_cOutFile2 << std::endl;
					
			}
		}
	}
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CBoilerplateLoopFunctions, "boilerplate_loop_functions");

}
