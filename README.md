# Centralization vs. decentralization in sweep coverage with ground robots and UAVs

This repository contains the Lua controllers and loop functions of five approaches to multi-robot sweep 
coverage, implemented in the ARGoS simulator (https://github.com/ilpincy/argos3).

## Configuring and running experiments

The default setups for each project contain 100 objects and 8 ground robots and in the Hybrid formation and 
Beacon-based approaches, 3 UAVs have been used. To run an approach with a set of objects, please follow
the steps below:
 
1- Place a blocks.csv file in the "src/boilerplate/" directory of the project; this file contains the positions and 
   orientations of the set of objects that should be placed in the environment. The number of the objects should be 
   also specified in src/boilerplate/boilerplate.argos. For example, if we want to distribute 200 objects, we should 
   put a blocks.csv file containing the positions and orientations of those 200 objects and set the "quantity" of objects 
   to 200 in the boilerplate.argos:

- In order to change the number of objects/blocks that should be placed in the environment, the parameter "quantity"
  in the block of code associated with the object prototype (which has been specified by comment "blocks" as header; 
  e.g., line 202 in MNS/src/boilerplate/boilerplate.argos, line 154 in Beacon-based decentralized/src/boilerplate/boilerplate.argos) 
  in the boilerplate.argos should be changed. 

- The output files have been defined under the output attribute of the loop function node (e.g., line 66 of MNS/src/boilerplate/boilerplate.argos). 
  As can be seen in the boilerplate.argos, output files have been defined for experiments, namely, output.csv, Median_time.csv, and blocks.csv. 
  For more information on these output files please read section [Output of experiment](#Output-of-experiment). It is worth noting that blocks.csv  
  is a copy of the blocks.csv file used in the experiment. Furthermore, in addition to these three .csv files, we record a copy of boilerplate.argos  
  file used in the experiment.

2- If someone wants to design a new setup based on his/her project, he/she needs to change the environmental properties 
   defined in the boilerplate.argos file. By following the comments in the file, we can change 
   the robot and UAV prototypes, the arena, the number of robots, or whatever is needed for the experiment in the 
   boilerplate.argos file.

3- After modifying the boilerplate.argos file, the loop function of the project (boilerplate_loop_functions.cpp), which is located 
   in the src/boilerplate/ directory, should also be edited accordingly. The parameters of the loop function are defined as follows:

- Nb_robots: the number of robots used in the experiment
- Nb_blocks: the number of blocks used in the experiment

By changing the number of blocks or robots in the boilerplate.argos file, the two relevant parameters should be adjusted in
the boilerplate_loop_functions.cpp file.


4- It is worth noting that in Hybrid formation and Centralized formation projects, in order to change the topology or 
   structure of the MNS, we need to modify the structure_quadcopter.lua. All the files have been sufficiently commented
   such that it is very easy to modify different parts of the code according to the goals of the research.


## Output of experiment

Each experiment generates four output files: Median_time.csv, output.csv, blocks.csv, and boilerplate.argos. The latter two 
are a copy of the blocks.csv and boilerplate.argos files used in the experiment, respectively. Median_time.csv is a 16 x 16 
matrix which records the total time spent by robots in each cell of the grid at the end of the experiment. Finally, output.csv 
is tab separated values. The values of this file are defined as follows:

- the number of messages sent by each UAV/robot at each time step; there is one column for this output per each UAV/robot
- the total number of messages sent within the swarm from the beginning of the experiment
- the total distance a UAV/robot has travelled from the beginning of the experiment; there is one column for this output per each UAV/robot
- the total distance travelled by the robots from the beginning of the experiment
- the total robot-robot collisions from the beginning of the experiment
- the coverage percentage from the beginning of the experiment
- the total number of visited cells from the beginning of the experiment
- the total number of unvisited cells from the beginning of the experiment
- the coverage uniformity from the beginning of the experiment
- the simulation time step


## Compilation and installation

To compile and run each project, follow these steps. 

First, checkout the correct version of ARGoS. The version of ARGoS that has been used for this project 
is "5101304c9f6b776458c822a37a973f598b08700c".
To clone this version of ARGoS:

	git clone https://github.com/ilpincy/argos3
	cd argos3
	git checkout 5101304c9f6b776458c822a37a973f598b08700c

To compile and install this version of ARGoS:

	mkdir build
	cd build
	cmake ../src
	make
	sudo make install

After making sure that the correct version has been installed, compile and install the project.

In the main directory of the approach project (i.e., the directory that contains the `src`), 
make a build directory, then compile and install the project:

	cd "project name"
	mkdir build
	cd build
	cmake ../src
	make
	sudo make install

To run the project, go to `src/boilerplate/` and run ARGoS:

	cd ../src/boilerplate
	argos3 -c boilerplate.argos

