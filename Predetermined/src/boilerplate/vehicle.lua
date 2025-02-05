----------------------------------------------------------------------------------
--   Global Variables
----------------------------------------------------------------------------------

require("PackageInterface")
--require("lfs")
local State = require("StateMachine")
--require("debugger")
local leftCountN = 0
local rightCountN = 0
local setCounterN = 0
local set = 1
local drivingPhase = 0
local circulating = 0
local stopCirCounter = 0
local  timeStepN = 0

fn = ''
global_folder = ''
fii = 0
local speeds_x = {}
local speeds_y = {}
local default_speed = 0
	-- create a folder per each run in order to record the speed values of robots over time separately
	global_folder = 'Speeds_data/'..os.date('%d-%m-%y_%H-%M-%S')
	os.execute("mkdir " .. global_folder)

----------------------------------------------------------------------------------
--   State Machine: the behavior of the robots and UAV are implemented based on a state machine
----------------------------------------------------------------------------------
----------------------------------------------------------------------------------------
stateMachine = State:create{
	initial = "randomWalk",	-- the intial state of the robots is set to random walk but in this project the robots are requrited in the first step
	substates = 
	{
		randomWalk = State:create{
			transMethod = function()
				local fromidS, cmdS, rxNumbersNT = getCMD()	-- red the info of the received message if there is any
				if cmdS == "recruit" then	-- when the robot is reqruited its state turns to "beingDriven": line 83
					setSpeed(0, 0)	-- set the speed to 0
					return "beingDriven"
				elseif cmdS == "wait" then	
					return "Pairing"
				end
			end,
			initial = "straight",
			substates = {
				straight = State:create{
					enterMethod = function() goFront() end,
					transMethod = function()
						if objFront() == true then
							standStill()
							return "turn"
						end
						sideForward((math.random() - 0.5) * 5)
					end,
				}, 
				turn = State:create{
					enterMethod = function()
						if math.random() > 0.5 then turnLeft()
						                       else turnRight() end
					end,
					transMethod = function()
						if objFront() == false then
							return "straight"
						end
					end,
				},
			},
		}, -- end of randomWalk
		Pairing = State:create{
			enterMethod = function() setSpeed(0, 0) print(getSelfIDS(), ": I am waiting for my partner") end,
			transMethod = function()
				local fromidS, cmdS, rxNumbersNT = getCMD()
				if cmdS == "go" then
					print('me?')
					return "beingDriven"
				end
			end,

		},
		-- when the robot is reqrited, it follows the following behavior
		beingDriven = State:create{
			data = {countN = 0},
			enterMethod = function() setSpeed(0, 0) print(getSelfIDS(), ": I am recruited") end,
			transMethod = function(fdata, data, para)
				print("PHASE_VEHICLE: ", getSelfIDS())
				local fromidS, cmdS, rxNumbersNT = getCMD()
				if cmdS == 'driving' then 
					drivingPhase = 1
				end
				if stopCirCounter > 0 then
					stopCirCounter = stopCirCounter - 1
				end
				-- General note for obstacle avoidance: getProximityN(i) returns a float value between 0 and 1 for proximity sensor i (for i = 1, 2, 3,..., 12); 
				-- in the predetermined approach, the length of a proximity sensor is 5 cm, so the values returned by this function for proximity sensor i
				-- is between 0.95 and 1 when it senses an obstacle; otherwise it returns 0;
				-- when the sensor fully overlaps with the obstacle (i.e., 5 cm; this means the robot has attached to the obstacle), the returned value is maximum (i.e., 1); 
				-- when it overlaps for 4 cm (this means the object is 1 cm away from the edge of the robot in the direction of the sensor) the returned value is 0.99; 
				-- when it overlaps for 3 cm (this means the object is 2 cm away from the edge of the robot in the direction of the sensor) the returned value is 0.98; 
				-- when it overlaps for 2 cm (this means the object is 3 cm away from the edge of the robot in the direction of the sensor) the returned value is 0.97; 
				-- when it overlaps for 1 cm (this means the object is 4 cm away from the edge of the robot in the direction of the sensor) the returned value is 0.96;
				-- when it overlaps for an epsilon value greater than 0 cm (this means the object is  a bit less than 5 cm away from the edge of the robot in the
				-- direction of the sensor), the returned value is a bit greater than 0.95;
				if drivingPhase == 0 then -- this is not used in this project; it is used when the robots are independent while in this project they are reqruited and control by the UAV
					print("drivingPhase", drivingPhase)
					-- when poximity sensor 2 senses an object, and it is within 2 cm of the edge of the robot in the direction of the sensor
					if (getProximityN(2) >= 0.98) then	
						setSpeed(9, 3)	-- left wheel's speed: 9, and right wheel's speed: 3 
					-- when poximity sensor 12 senses an object, and it is within 2 cm of the edge of the robot in the direction of the sensor
					elseif (getProximityN(12) >= 0.98) then	
						setSpeed(3, 9)	-- left wheel's speed: 3, and right wheel's speed: 9 
					elseif getProximityN(1) ~=0 then	-- when poximity sensor 1 senses an object
						setSpeed(7, 3)	-- left wheel's speed: 7, and right wheel's speed: 3 
					elseif getProximityN(12) ~=0 then	-- when poximity sensor 12 senses an object
						setSpeed(5, 7)	-- left wheel's speed: 5, and right wheel's speed: 7 					
					elseif getProximityN(2) ~=0 then	-- when poximity sensor 2 senses an object
						setSpeed(7, 5)	-- left wheel's speed: 7, and right wheel's speed: 5 						
					elseif getProximityN(3) ~=0 then	-- when poximity sensor 3 senses an object
						setSpeed(7, 5)	-- left wheel's speed: 7, and right wheel's speed: 5 	
					elseif getProximityN(11) ~=0 then	-- when poximity sensor 11 senses an object
						setSpeed(5, 7)	-- left wheel's speed: 5, and right wheel's speed: 7 
					elseif getProximityN(10) ~=0 then	-- when poximity sensor 10 senses an object
						setSpeed(9, 9)	-- left wheel's speed: 9, and right wheel's speed: 9 
					elseif getProximityN(4) ~=0 then	-- when poximity sensor 4 senses an object
						setSpeed(9, 9)	-- left wheel's speed: 9, and right wheel's speed: 9 
					elseif cmdS == "setspeed"  then
						-- left wheel's speed: rxNumbersNT[1] (sent by the UAV), and right wheel's speed: rxNumbersNT[2] (sent by the UAV):
						setSpeed(rxNumbersNT[1], rxNumbersNT[2])
					end
				elseif drivingPhase == 1 then
					-- when the robot successfully passes an obstacle with following a predetermined counterclockwise half-circle trajectory, 
					-- the UAV informs it with a "stopCirculating" message
					-- based on this message the robot stops ciculating around the obstacle and ajusts its direction to move forward towars the target point
					-- to this end, the robot changes its status: lines 136 to 139
					if (cmdS == "stopCirculating") then 
						drivingPhase = 2	-- change the status in order to keep sweeping the lane
						stopCirCounter = 30	-- a counter during which the ground robot avoids circulating any obstacle
						circulating = 0		
					-- te robot follows a predetermined counterclockwise half-circle trajectory around the obstacle based on the following set of sensory readings-movemnt:
					-- lines 142 to 168
					elseif  stopCirCounter == 0 then	 
						if (getProximityN(1) >= 0.97) then
							setSpeed(10, -10)	-- left wheel's speed: 10, and right wheel's speed: -10
							if circulating == 0 then
								circulating = 1	-- this flag is used to contol the process of following the predetermined counterclockwise half-circle trajectory around the obstacle
							end
						-- to avoid passing through two obstacles when the distance between them is smaller than the width of a ground robot
						elseif ((getProximityN(12) >= 0.97 or getProximityN(11) >= 0.97) and getProximityN(1) == 0 and (getProximityN(2) ~= 0 or getProximityN(3) ~= 0)) then					
							setSpeed(2,-2)	-- left wheel's speed: 2, and right wheel's speed: -2
						elseif (getProximityN(2) ~= 0  or getProximityN(3) ~= 0 ) then
							if getProximityN(3) >= 0.99 or getProximityN(2) >= 0.99 then
								setSpeed(-4,-4)	-- left wheel's speed: -4, and right wheel's speed: -4
							elseif getProximityN(2) < 0.99 then
								setSpeed(9,-3)	-- left wheel's speed: 9, and right wheel's speed: -3
							elseif getProximityN(3) < 0.99 and circulating == 1 then
								setSpeed(9,-3)	-- left wheel's speed: 9, and right wheel's speed: -3
							elseif getProximityN(3) >= 0.97 and circulating ~= 1 then
								setSpeed(9,3)	-- left wheel's speed: 9, and right wheel's speed: 3
							end
						
							if circulating == 0 and (getProximityN(2) >= 0.99) then	
								circulating = 1
							end
						elseif (getProximityN(4) ~= 0) then
							setSpeed(5,10)	-- left wheel's speed: 5, and right wheel's speed: 10
						end
					end
					-- when the robot is not following the counterclockwise half-circle trajectory, under following conditions it smouthly passes the obstacle it encounters with:
					-- lines 171 to 187
					if cmdS == "setspeed" and circulating == 0 then
						if getProximityN(3) >= 0.97 then	-- this is a redundant condition to make sure the robot does not stuck behind an obstacle
							setSpeed(9,3)	-- left wheel's speed: 9, and right wheel's speed: 3
						-- under these conditions, the robot smouthly passes the obstacle by turning left (lines 175 to 186)				
						elseif (getProximityN(12) >= 0.97 and getProximityN(11) >= 0.97 and getProximityN(1)==0) then
							setSpeed(3,9)	-- left wheel's speed: 3, and right wheel's speed: 9
						elseif (getProximityN(11) >= 0.96 and getProximityN(1)==0) then	-- to smouthly passes the obstacle by turning left
							setSpeed(4,7)	-- left wheel's speed: 4, and right wheel's speed: 7	
						elseif (getProximityN(12) >= 0.96 and getProximityN(1)==0) then	-- to smouthly passes the obstacle by turning left
							setSpeed(3,10)	-- left wheel's speed: 3, and right wheel's speed: 10
						--elseif (getProximityN(12) >= 0.99 and getProximityN(1)==0) then
							--setSpeed(-2,-2)
						else
							-- left wheel's speed: rxNumbersNT[1] (sent by the UAV), and right wheel's speed: rxNumbersNT[2] (sent by the UAV):
							setSpeed(rxNumbersNT[1], rxNumbersNT[2])
						end
					end
				-- the robot adjusts its direction when it receives a "stopCirculating" message	
				elseif drivingPhase == 2 then
					setSpeed(2, -2)	-- left wheel's speed: 2, and right wheel's speed: -2
					if cmdS == "normal" then
						drivingPhase = 1
					end
				end
				if cmdS == "dismiss" then
					print(getSelfIDS(), ": I am dismissed")
					return "randomWalk"
				end
				if fromidS ~= nil then
					data.countN = 0
				else
					-- i didn't get command when I should be
					data.countN = data.countN + 1
					if data.countN > 3 then
						print(getSelfIDS(), ": I am lost")
						return "randomWalk"
					end
				end
			end,
		}, -- end of beingDriven
	} -- end of substates of stateMachine
} -- end of stateMachine

----------------------------------------------------------------------------------
--   ARGoS Functions
----------------------------------------------------------------------------------
function init()
	setTag(getSelfIDS())
	reset()
end

-------------------------------------------------------------------
function step()
	timeStepN =  timeStepN + 1
	if (timeStepN == 1) then
		fn=os.date(global_folder..'/%d-%m-%y_%H-%M-%S_'..getSelfIDS()..'.csv')	-- create a csv file per each robot to record its speed values over time
		print(fn)
		
		setSpeed(0, 0)	-- set the speed to 0
	end
	
	stateMachine:step()
	local fromidS, cmdS, rxNumbersNT = getCMD()
	-- lines 235 to 245 are for reading the id of the faulty robots
	local fault_flag = 0
	if timeStepN >= 500 then
		for line in lines do
			--print("\t" .. line)
			local ID = 'vehicle'..line..'0'
			if (getSelfIDS() == ID) then
				print(ID)
				fault_flag = 1
			end
		end
	end

	if speeds_x[timeStepN] == nil or speeds_y[timeStepN] == nil then
		if fault_flag == 1 then
			setSpeed(0, 0)	-- if the robot is faulty, set the speed to 0 
		else
			-- if the robot do not receive any speed value from the UAV and is not sensing any obstacle in its front:
			-- it sets its left wheel and right wheel speeds to those of the previous step, repectively
			speeds_x[timeStepN] = speeds_x[timeStepN-1]
			speeds_y[timeStepN] = speeds_y[timeStepN-1]
			setSpeed(speeds_x[timeStepN], speeds_y[timeStepN])	
		end

	end
	-- record the left wheel speed and right wheel speed of the robot in addition to the robots ID and the relevant time step		
	fii = io.open (fn,"a" )
	fii:write(getSelfIDS().. "\t")
	fii:write(timeStepN.. "\t")
	fii:write(speeds_x[timeStepN].. "\t")
	fii:write(speeds_y[timeStepN].. "\n")	
	fii:close()

end

-------------------------------------------------------------------
function reset()
	math.randomseed(1)
end

-------------------------------------------------------------------
function destroy()
   -- put your code here
end

----------------------------------------------------------------------------------
--   Customize Functions
----------------------------------------------------------------------------------
function standStill() -- not used in this project (called by random walk if no UAV recruits the ground robot)
end
function goFront() -- not used in this project (called by random walk if no UAV recruits the ground robot)
end
function turnLeft() -- not used in this project (called by random walk if no UAV recruits the ground robot)
end
function turnRight() -- not used in this project (called by random walk if no UAV recruits the ground robot)
end
function sideForward(x) -- not used in this project (called by random walk if no UAV recruits the ground robot)
end

-------------------------------------------------------------------
function objFront()
	if getProximityN(1) ~= 0 or
	   getProximityN(2) ~= 0 or
	   getProximityN(12) ~= 0 then
		return true
	else
		return false
	end
end

-------------------------------------------------------------------
function getCMD()
	for i, rxBytesBT in pairs(getReceivedDataTableBT()) do	-- byte table
		local toIDS, fromIDS, cmdS, rxNumbersNT = bytesToTable(rxBytesBT)
		if toIDS == getSelfIDS() then
			return fromIDS, cmdS, rxNumbersNT
		end
	end
end

function sendCMD(toidS, cmdS, txDataNT)
	local txBytesBT = tableToBytes(toidS, 
	                               getSelfIDS(), 
                                   cmdS,
                                   txDataNT)
	transData(txBytesBT)
end

----------------------------------------------------------------------------------
--   Lua Interface
----------------------------------------------------------------------------------
function setSpeed(x,y)
	print(getSelfIDS())
	print(fn)
	local fault_flag2 = 0
	local faults = io.open ("FaultTolerance/faulty_robots.csv","r" )
	lines = faults:lines()
	-- lines 332 to 340 are for reading the id of the faulty robots
	if timeStepN >= 500 then
		for line in lines do
			local ID = 'vehicle'..line..'0'
			if (getSelfIDS() == ID) then
				print(ID)
				fault_flag2 = 1
			end
		end
	end
	-- if the robot is faulty, set the speed to 0 : lines 342 to 347
	if (fault_flag2 == 1) then	  
		x = 0
		y = 0
	end	
	speeds_x[timeStepN] = x
	speeds_y[timeStepN] = y
	
	-- set left wheel and right weel speeds
	robot.joints.base_wheel_left.set_target(x)
	robot.joints.base_wheel_right.set_target(-y)

end

function getProximityN(x)
	return robot.proximity[x]
end

function getProximityTableNT()
	return robot.proximity
end

function transData(xBT)
	robot.radios["radio_0"].tx_data(xBT)
end

function getReceivedDataTableBT()	--BT means byte table
	return robot.radios["radio_0"].rx_data
end

function getSelfIDS()
	return robot.id
end

function setTag(str)
	robot.tags.set_all_payloads(str)
end
