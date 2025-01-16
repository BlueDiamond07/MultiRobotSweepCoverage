-- Driver --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Driver = {VNSMODULECLASS = true}
Driver.__index = Driver
local turn_dir
function Driver:new()
	local instance = {}
	setmetatable(instance, self)
	instance.lastReceivedSpeed = {
		locV3 = Vec3:create(),
		dirV3 = Vec3:create(),
	}
	return instance
end

function Driver:deleteParent(vns)
	self.lastReceivedSpeed = {
		locV3 = Vec3:create(),
		dirV3 = Vec3:create(),
	}
end

function Driver:run(vns, paraT)
	local randomflag
	local flag = 0
	if (paraT.stepCounter == 1) then -- initialaize the turn direction randomly
		local random_number = math.random()
		if random_number < 0.5 then
			turn_dir = 1 -- in order to turn left
		else
			turn_dir = -1 -- in order to turn right
		end
	elseif (paraT.stepCounter%200 == 0) then -- change the turn direction once per every 200 steps 
		turn_dir = -1*turn_dir
	end

	-- listen to drive from parent
	message_flag = 0
	for _, msgM in pairs(vns.Msg.getAM("ALLMSG", "drive")) do
			message_flag = 1
			-- calculate the robot's direction (i.e., the relevant vector)
			relRobotDir = {x = msgM.dataT.front.x - msgM.dataT.locV3.x, y = msgM.dataT.front.y - msgM.dataT.locV3.y,}
			-- calculate the vector from the robot to the UAV			
			relRobotToQuadcopter = {x = 0 - msgM.dataT.locV3.x, y = 0 - msgM.dataT.locV3.y,}
			-- calculate the length of the robot's direction vector
			robotQuadcopterVector_dis = math.sqrt(relRobotDir.x * relRobotDir.x + relRobotDir.y * relRobotDir.y )
			-- calculate the length of the vector from robot to the UAV
			robotVector_dis = math.sqrt(relRobotToQuadcopter.x * relRobotToQuadcopter.x + relRobotToQuadcopter.y * relRobotToQuadcopter.y)
			-- calculate the angle between the robot's heading (in radian) and the UAV (i.e., the robot's orentation in the reference frame of the UAV)
			angle_radian = math.acos(((relRobotToQuadcopter.x*relRobotDir.x)+(relRobotToQuadcopter.y*relRobotDir.y)) / (robotVector_dis * robotQuadcopterVector_dis))
	end

	-- add random speed
	if vns.randomWalkerSpeed ~= nil then
		local scalar = 1	
		transV3 = vns.randomWalkerSpeed.locV3 * scalar
		rotateV3 = rotateV3 + vns.randomWalkerSpeed.dirV3 * scalar
		randomflag = 1
	end
	

	if vns.robotType == "vehicle" then
		local scalar = 1.2
		-- apply the box avoider speed from BoxAvoider.lua
		if vns.boxAvoiderSpeed ~= nil then
			transV3 = vns.boxAvoiderSpeed.locV3
		else
			transV3 = transV3
		end
	end
	
	if (message_flag == 1 and math.deg(angle_radian)< 90) then -- if the angle between the robot's heading and the UAV is less than 90 degrees
		if vns.boxAvoiderSpeed ~= nil then -- if an obstacle is sensed with the front sensors; the priority is with obstacle avoidance
			transV3 = vns.boxAvoiderSpeed.locV3
			vns.move(transV3, rotateV3, paraT.stepCounter, randomflag)
		else -- if no obstacle is sensed with the front sensors
			if (turn_dir == 1) then
				vns.setSpeed(-4.5,7) -- turn left (left wheel speed: -4.5 and right wheel speed: 7)
			else
				vns.setSpeed(7,-4.5) -- turn right (left wheel speed: 7 and right wheel speed: -4.5)
			end
		end
	else -- follow the random walk motion strategy
		vns.move(transV3, rotateV3, paraT.stepCounter, randomflag) -- move function is called
	end

	if (vns.robotType == "quadcopter") then
		-- to count the nuumber of observed robots	
		local number_of_robots = 0 -- a counter for the number of robots in the UAV's FOV
		-- for each observed robot increase the counter 
		for _, robotVns in pairs(paraT.vehiclesTR) do
			number_of_robots = number_of_robots + 1
		end

		-- if the number of obsreved robots is higher that 25% of the number of ground robots in the swarm (i.e., more than 2 in the default setup where 8 ground rorbots are used)
		if (number_of_robots > 2) then
			for _, robotVns in pairs(paraT.vehiclesTR) do				
				-- send the observed robot the coordinates of its center and heading; the latter one allows the robots to calculate its orentation in the reference frame of the UAV 
				vns.Msg.send(robotVns.idS, "drive", {locV3 = robotVns.locV3, front = robotVns.front})
			end
		end

	end
end


return Driver

