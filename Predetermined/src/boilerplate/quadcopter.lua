----------------------------------------------------------------------------------
--   Global Variables
----------------------------------------------------------------------------------

require("PackageInterface")

local STATE = "recruiting"
local SON_VEHICLE_NAME = {}
local timeStepN = 0
local straightForward = 7	-- the speed of the robots are set to 7 cm/s when they move forward (the UAV sends the speeds to the robots)
local lunch = {}
local allocation = 1
local targets = {}
local nextTarget = {}
local targetsDone = {}
local avoidanceControl = {}
local difTargetsControl = {}
local ACReactivation = {}
local tempX = {}
local tempY = {}
local tempDis = {}
local drivingSet = {}
local escape = {}
local CirculateCounter = {}
----------------------------------------------------------------------------------
--   ARGoS Functions
----------------------------------------------------------------------------------
function init()
	reset()
end

-------------------------------------------------------------------
function step()
	timeStepN = timeStepN + 1
	-- detect boxes into boxesVT[i]	
	local boxesVT = getBoxesVT()	
		-- V means vector = {x,y}
	-- detect robots into robotsRT[i] and robotsRT[id]
	local robotsRT = getRobotsRT()
		-- R for robot = {locV, dirN, idS}
	local length = #robotsRT	-- lenght is set to the number of robots
	local numRegions = #robotsRT
	
	if allocation == 1 then
		for i=1, #robotsRT do
			difTargetsControl[i] = 0
			tempX[i] = 0
			tempY[i] = 0
			tempDis[i] = 0
			avoidanceControl[i] = 0
			ACReactivation[i] = 0
		end
		-- decompose the arena through dividing its side length by the number of ground robots and
		-- calculate the x coordinate of the lunch points:
		local sideDecompScale = 400/length
		local initialLoc = -187.5
		local lastLoc = 200
		local mapArray = {}
		for i=1, #robotsRT do
			mapArray[i] = {}
			for j=1, 2 do
				mapArray[i][j] = initialLoc 
				initialLoc = initialLoc + 50 --sideDecompScale	
			end
			initialLoc = initialLoc - 50 --sideDecompScale
		end
		-- assign each area to the nearest robot: 
		local alreadySelected = {}
		local count = 1
		for i=1, length do -- for each lunch point
			targets[i] = {}
			lunch[i] = {}
			local minDis = 1000
			local closestRobotToInit = nil
			for j=1, #robotsRT do -- for each ground robot
				local inFlag = 0
				if #alreadySelected~=0 then
					for k=1, #alreadySelected do
						if (alreadySelected[k] == robotsRT[j].idS) then
							inFlag = 1
						end
					end
				else
					inFlag = 0
				end
				if (inFlag == 0) then
					-- Distance Computing: between the selected robot and the selected lunch point
					local relativeV = {x = robotsRT[j].locV.x - mapArray[i][1],
								   y = robotsRT[j].locV.y - mapArray[i][2],}
					local disToMapInits = math.sqrt(relativeV.x * relativeV.x +
									relativeV.y * relativeV.y )
					if (disToMapInits < minDis) then
						minDis = disToMapInits
						closestRobotToInit = robotsRT[j].idS
					end
				end
			end
			alreadySelected[count] = closestRobotToInit
			count = count + 1
			-- the following is usable for initialization phase and when the robots are far from their intial points. In this project they are placed manually in their initial lcoations (a point that is a few centimeters outside the arena)
			lunch[closestRobotToInit] = {closestRobotToInit, {x = mapArray[i][1], y = -200}}
			 -- assign 3 target points to the ground robot to simulate the boustrophedon motion in this particular area:
			targets[lunch[closestRobotToInit][1]] = { 
				{visited=0, x=lunch[closestRobotToInit][2].x, y=197}, 
				{visited=0, x=lunch[closestRobotToInit][2].x+23, y=197}, 
				{visited=0, x=lunch[closestRobotToInit][2].x+23, y=-212}
			}
		end
		for i = 1, #robotsRT do
			targetsDone[i] = 0
		end
		allocation = 0
	end
	-- recruit ground robots
	if STATE == "recruiting" then
		for i=1, #robotsRT do	  -- for each robot
			sendCMD(robotsRT[i].idS, "recruit")	-- this message is sent to the robot to recruit it in the first step
			print(getSelfIDS(), ": i recruit vehicle", robotsRT[i].idS)
		end
		for i=1, #robotsRT do	  -- for each robot
			SON_VEHICLE_NAME[i] = robotsRT[i].idS
			difTargetsControl[SON_VEHICLE_NAME[i]] = 0
			tempX[SON_VEHICLE_NAME[i]] = 0
			tempY[SON_VEHICLE_NAME[i]] = 0
			tempDis[SON_VEHICLE_NAME[i]] = 0
			avoidanceControl[SON_VEHICLE_NAME[i]] = 0
			drivingSet[SON_VEHICLE_NAME[i]] = 0
			CirculateCounter[SON_VEHICLE_NAME[i]] = 300	  -- a counter to stop a circulation procedure which looks unnormal 
		end
		STATE = "driving"
		return -- return of step()
	elseif STATE == "driving" then -- drive the recruited ground robots
		for i = 1, #robotsRT do -- for each ground robot
			-- each ground robot should be guided towards its next target point in its
			-- assigned area as long as its targetsDone flag is 0
			if targetsDone[i] == 0 then	
				local min = 10000
				for j=1, #targets[SON_VEHICLE_NAME[i]] do
					if targets[SON_VEHICLE_NAME[i]][j].visited == 0 and j<min then
						nextTarget[SON_VEHICLE_NAME[i]]=targets[SON_VEHICLE_NAME[i]][j]
						min = j
					end
				end
				-- to change the state of the ground robot when it reaches its target lunch point
				if min == 1 and drivingSet[SON_VEHICLE_NAME[i]] == 0 then
					sendCMD(robotsRT[SON_VEHICLE_NAME[i]].idS, "driving")	-- this message is sent to robot to change its status in order to be deriven by the UAV
					drivingSet[SON_VEHICLE_NAME[i]] =  1
				
				elseif nextTarget[SON_VEHICLE_NAME[i]] ~= nil then
					-- calculate the direction and distance of the ground robot with respect to its next target point
					local lunchDirtoRobotN = getLunchDirtoRobot(robotsRT[SON_VEHICLE_NAME[i]].locV, {x=nextTarget[SON_VEHICLE_NAME[i]].x,y=nextTarget[SON_VEHICLE_NAME[i]].y})
					local difTargetsN = lunchDirtoRobotN - robotsRT[SON_VEHICLE_NAME[i]].dirN
					while difTargetsN > 180 do
						difTargetsN = difTargetsN - 360
					end

					while difTargetsN < -180 do
						difTargetsN = difTargetsN + 360
					end
					local relV = {x = robotsRT[SON_VEHICLE_NAME[i]].locV.x - nextTarget[SON_VEHICLE_NAME[i]].x,
					              y = robotsRT[SON_VEHICLE_NAME[i]].locV.y - nextTarget[SON_VEHICLE_NAME[i]].y,}
					local disToTarget = math.sqrt(relV.x * relV.x + relV.y * relV.y )
					-- Note: circulation procedure is the prcidure by which a robot follows a predetermined counterclockwise half-circle trajectory (please see vehicle.lua)
					if escape[SON_VEHICLE_NAME[i]] == 0 or escape[SON_VEHICLE_NAME[i]] == nil then
						if (avoidanceControl[SON_VEHICLE_NAME[i]] == 0 and difTargetsControl[SON_VEHICLE_NAME[i]] ~= nil and (difTargetsControl[i] - difTargetsN) < -40 and robotsRT[SON_VEHICLE_NAME[i]].locV.y >= -200 and robotsRT[SON_VEHICLE_NAME[i]].locV.y <= 200) then -- record the postion of the ground robots when it meets an obstacle and set a timer for the circulation procedure (when the UAV detects that the difference between the direction of the ground robot and its target direction is less than -40 degrees, it realizes that the ground robot has encounted an obstacle)
							avoidanceControl[SON_VEHICLE_NAME[i]] = 1 -- a flag to control avoidance
							tempX[SON_VEHICLE_NAME[i]] = robotsRT[SON_VEHICLE_NAME[i]].locV.x
							tempY[SON_VEHICLE_NAME[i]] = robotsRT[SON_VEHICLE_NAME[i]].locV.y
							CirculateCounter[SON_VEHICLE_NAME[i]] = 300  -- a counter to stop a circulation procedure which looks unnormal 
						end
						if disToTarget > 2 then -- when the robot has not yet reached the next target point

							if difTargetsN > 3 or difTargetsN < -3 then -- adjust the direction of the ground robot with respect to its next target point in this range
								if avoidanceControl[SON_VEHICLE_NAME[i]] == 0 or min==0 then
									if  min == 0 then -- just for the lunch ponit
										if (difTargetsN > 0) then
											-- set left wheel speed and right wheel speed of the robot to -1 and 1, respectively
											-- this speed lets the robot turn towards the next target point in a more accurate way
											setRobotVelocity(SON_VEHICLE_NAME[i], -1, 1) 
										else
											-- set left wheel speed and right wheel speed of the robot to 1 and -1, respectively
											-- this speed lets the robot turn towards the next target point in a more accurate way
											setRobotVelocity(SON_VEHICLE_NAME[i], 1, -1) 
										end
									else
										if (difTargetsN > 0) then
											-- set left wheel speed and right wheel speed of the robot to 3 and 10, respectively
											-- this speed lets the robot turn towards the next target point in a more accurate way
											setRobotVelocity(SON_VEHICLE_NAME[i], 3, 10) 
										else
											-- set left wheel speed and right wheel speed of the robot to 10 and 3, respectively
											-- this speed lets the robot turn towards the next target point in a more accurate way
											setRobotVelocity(SON_VEHICLE_NAME[i], 10, 3) 
										end
									end
								else
									CirculateCounter[SON_VEHICLE_NAME[i]] = CirculateCounter[SON_VEHICLE_NAME[i]] - 1
									if nextTarget[SON_VEHICLE_NAME[i]].y > 0 then
										difOfY = robotsRT[SON_VEHICLE_NAME[i]].locV.y - tempY[SON_VEHICLE_NAME[i]]
									else
										difOfY = tempY[SON_VEHICLE_NAME[i]] - robotsRT[SON_VEHICLE_NAME[i]].locV.y
									end
									if (tempX[SON_VEHICLE_NAME[i]] ~= nil and tempY[SON_VEHICLE_NAME[i]] ~= nil) then
										 -- stop the circulation procedure when the robot approximately returns to the line it was in when it detected the 											 -- obstacle before starting its circulation procedure 
										if (CirculateCounter[SON_VEHICLE_NAME[i]] == 0 or (math.abs(robotsRT[SON_VEHICLE_NAME[i]].locV.x - tempX[SON_VEHICLE_NAME[i]])< 0.25 and difOfY > 0.4)) then
											sendCMD(robotsRT[SON_VEHICLE_NAME[i]].idS, "stopCirculating")  -- to ask the robot to stop the circulation procedure
											escape[SON_VEHICLE_NAME[i]] = 1
											avoidanceControl[SON_VEHICLE_NAME[i]] = 0
										else
											setRobotVelocity(SON_VEHICLE_NAME[i], -2, 2)	-- set left wheel speed and right wheel speed of the robot to -2 and 2, respectively
										end
									end
								end
							else	-- it is not required to change the direction of the ground robot (the robot is moving toward its next target point)
								if avoidanceControl[SON_VEHICLE_NAME[i]] == 0 then
									setRobotVelocity(SON_VEHICLE_NAME[i], straightForward, straightForward) -- set left wheel speed and right wheel speed of the robot to 7 and 7, respectively
									if min >= 1 then
										difTargetsControl[SON_VEHICLE_NAME[i]] = difTargetsN
									end
								else
									CirculateCounter[SON_VEHICLE_NAME[i]] = CirculateCounter[SON_VEHICLE_NAME[i]] - 1
									setRobotVelocity(SON_VEHICLE_NAME[i], straightForward, straightForward) -- set left wheel speed and right wheel speed of the robot to 7 and 7, respectively
								end
							end
						elseif disToTarget <= 2 then -- when the robot reaches the next target point
							targets[SON_VEHICLE_NAME[i]][min].visited = 1
							difTargetsControl[SON_VEHICLE_NAME[i]] = nil
							setRobotVelocity(SON_VEHICLE_NAME[i], 0, 0)	-- set left wheel speed and right wheel speed of the robot to 0 and 0, respectively
							if targets[SON_VEHICLE_NAME[i]][#targets[SON_VEHICLE_NAME[i]]].visited == 1 then
								targetsDone[i] = 1
							end
						end --end of disToTarget
					elseif escape[SON_VEHICLE_NAME[i]] == 1 then -- end the circulation procedure
						if difTargetsN > 5 or difTargetsN < -5 then
							if (difTargetsN > 0) then
								setRobotVelocity(SON_VEHICLE_NAME[i], -2, 2)	 -- set left wheel speed and right wheel speed of the robot to -2 and 2, respectively
							else
								setRobotVelocity(SON_VEHICLE_NAME[i], 2, -2)	-- set left wheel speed and right wheel speed of the robot to 2 and -2, respectively
							end
						else
							sendCMD(robotsRT[SON_VEHICLE_NAME[i]].idS, "normal") -- to let the robot know that the situation is normal again which happens after stopping the robot from circulating
							escape[SON_VEHICLE_NAME[i]] = 0
						end
					end-- end of escape
				end --end of nextTarget ~= nil
			else -- make the ground robot stay stationary
				setRobotVelocity(SON_VEHICLE_NAME[i], 0, 0)	-- set left wheel speed and right wheel speed of the robot to 0 and 0, respectively
			end -- end of targetsDone
		end
	end-- two ends of STATE == "recruting" and "driving"
end

-------------------------------------------------------------------
function reset()
	local STATE = "recruiting"
end

-------------------------------------------------------------------
function destroy()
	-- put your code here
end

----------------------------------------------------------------------------------
--   Customize Functions
----------------------------------------------------------------------------------
function setRobotVelocity(id, x,y)
	local bytes = tableToBytes(id, robot.id , "setspeed", {x,y})
	robot.radios["radio_0"].tx_data(bytes)
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

-------------------------------------------------------------------
-- get boxes
-------------------------------------------------------------------
function getBoxesVT()
	local boxesVT = {}   -- vt for vector, which a table = {x,y}
	for i, detectionT in ipairs(getLEDsT()) do	
		-- a detection is a complicated table
		-- containing location and color information
		boxesVT[i] = getBoxPosition(detectionT)
	end	
	return boxesVT
end

function getBoxPosition (detection)
	local pos = {}
	pos.x = detection.center.x - 320
	pos.y = detection.center.y - 240
	pos.y = -pos.y 				-- make it left handed coordination system
	return pos
end

-------------------------------------------------------------------
-- get robots
-------------------------------------------------------------------
function getRobotsRT()
	robotsRT = {}
	for i, tagDetectionT in pairs(getTagsT()) do
		-- a tag detection is a complicated table
		-- containing center, corners, payloads
		
		-- get robot info
		local locV, dirN, idS = getRobotInfo(tagDetectionT)
		local headV = getRobotHead(tagDetectionT)
			-- locV V for vector {x,y}
			-- loc (0,0) in the middle, x+ right, y+ up , 
			-- dir from 0 to 360, x+ as 0
		local lunch  = {}
		robotsRT[i] = {locV = locV, headV = headV, dirN = dirN, idS = idS, lunch = nil}
		robotsRT[idS] = robotsRT[i]
	end
	return robotsRT
end

function getRobotInfo(tagT)
	-- a tag is a complicated table with center, corner, payload
	local degN = calcRobotDir(tagT.corners)
		-- a direction is a number from 0 to 360, 
		-- with 0 as the x+ axis of the quadcopter
		
	local locV = {}
	locV.x = tagT.center.x - 320
	locV.y = tagT.center.y - 240
	locV.y = -locV.y 				-- make it left handed coordination system

	local idS = tagT.payload

	return locV, degN, idS
end

-------------------------------------------------------------------
-- calculations
-------------------------------------------------------------------
function calcRobotDir(corners)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local front = {}
	front.x = (corners[3].x + corners[4].x) / 2
	front.y = -(corners[3].y + corners[4].y) / 2
	local back = {}
	back.x = (corners[1].x + corners[2].x) / 2
	back.y = -(corners[1].y + corners[2].y) / 2
	local deg = calcDir(back, front)
	return deg
end

-------------------------------------------------------------------
function getRobotHead(tag)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local pos = {}
	pos.x = ((tag.corners[3].x + tag.corners[4].x) / 2) - 640
	pos.y = ((tag.corners[3].y + tag.corners[4].y) / 2) - 480
	pos.y = -pos.y 				-- make it left handed coordination system
	return pos
end

-------------------------------------------------------------------
function getBoxDirtoRobot(robotPos, boxPos)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local deg = calcDir(robotPos, boxPos)
	return deg
end

-------------------------------------------------------------------
function getLunchDirtoRobot(center, target)
	-- from center{x,y} to target{x,y} in left hand
	-- calculate a deg from 0 to 360, x+ is 0
	local x = target.x - center.x
	local y = target.y - center.y
	local deg = math.atan(y / x) * 180 / 3.1415926
	if x < 0 then
		deg = deg + 180
	end
	if deg < 0 then
		deg = deg + 360
	end
	return deg
end

-------------------------------------------------------------------
function calcDir(center, target)
	-- from center{x,y} to target{x,y} in left hand
	-- calculate a deg from 0 to 360, x+ is 0
	local x = target.x - center.x
	local y = target.y - center.y
	local deg = math.atan(y / x) * 180 / 3.1415926
	if x < 0 then
		deg = deg + 180
	end
	if deg < 0 then
		deg = deg + 360
	end
	return deg
end

----------------------------------------------------------------------------------
--   Lua Interface
----------------------------------------------------------------------------------
function setVelocity(x,y,theta)
	robot.joints.axis0_axis1.set_target(x)
	robot.joints.axis1_axis2.set_target(y)
	robot.joints.axis2_body.set_target(theta)
end

function getLEDsT()
	return robot.cameras.fixed_camera.led_detector
end

function getTagsT()
	return robot.cameras.fixed_camera.tag_detector
end

function transData(xBT)		--BT means byte table
	robot.radios["radio_0"].tx_data(xBT)
end

function getReceivedDataTableBT()
	return robot.radios["radio_0"].rx_data
end

function getSelfIDS()
	return robot.id
end
