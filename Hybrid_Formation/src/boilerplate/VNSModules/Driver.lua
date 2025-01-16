-- Driver --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Driver = {VNSMODULECLASS = true}
Driver.__index = Driver
local tmpVector = {}
local prx10 = {}
local prx4 = {}
local counter = -1
l_timer = 0	-- a timer used for managing the motion of a ground robot with respect to its target point
r_timer = 0	-- a timer used for managing the motion of a ground robot with respect to its target point
my_Delay = 0	-- used to control the zig-zag motion: a counter used to make and keep border detection inactive for a certain period
Mytimer = 0	-- a timer during which the MNS waits for 200 steps when a border is detected, then shifts for a certain period depending on the length of the formation, and then waits for 100 steps after compeleting the shift (see line 233)
lock = 0	-- a flag used to manage the function that is called for moving the UAVs

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
		lock = 0
		adjust_R = 0	-- this flag is used to adjust the position of the robot with respect to its target point
		adjust_L = 0	-- this flag is used to adjust the position of the robot with respect to its target point
		vns.normal = 0	-- this flag is used for obstacle avoidance when the MNS is shifting to left or right
		flagAvoid = 0	-- this falg is used to control robot-robot collisions when they are avoiding obstacles

		if l_timer > 0 then
			l_timer = l_timer -1
		end
		if r_timer > 0 then
			r_timer = r_timer -1
		end
		if my_Delay > 0 then
			my_Delay = my_Delay -1
		end

		if Mytimer > 0 then
			Mytimer = Mytimer - 1
			if Mytimer == 0 then
				my_Delay = 1000
			elseif Mytimer == 1 then
				if vns.reverseMove == 0 then
					vns.reverseMove = 1
				elseif vns.reverseMove == 1 then
					vns.reverseMove = 0
				end
				for _, child in pairs(vns.childrenTVns) do
					-- the movement direction of the brain is sent to the other UAVs and ground robots; useful for the non-brain UAVs for managing the robot-robot collisions
					if (vns.idS == vns.brainS and child.robotType == "quadcopter") then
						vns.Msg.send(child.idS, "reverseMove", vns.reverseMove)
					end
				end
			end
		end
		
		local randomflag
		local flag = 0
		counter = counter + 1
		local sum = 0
		if vns.robotType == "vehicle" then
			for i=1, 12 do
				if i<=4 or i>=10 then	-- to check if the front half of the ground robot senses an obstacle or not
					sum = sum + paraT.proxTR[i]
				end
			end
			-- in lines 87 to 117 the right and left proximity sensors' data reading (proximity sensor number 10 and 4, respectively) are recorded in two separate arrays
			-- based on the data readings of the mentioned two sensors, a counter with length of 10 steps is defined: if for 30 steps, each of these two sensors does not sense any obstacle, a timer of length 10 is set
		        -- these timers are used by the BoxAvoider module (i.e., BoxAvoider.lua): the mentiond module simulates a half-circle trajectory (from left side or right side of the obstacle) for the corresponding ground robot to pass the obtacle during this time
			-- the priority (lines 113 to 117) is with proximity sensor 4 which leads to a half-circle trajectory from right side of the obstacle	
			if paraT.proxTR[10] ~= 0 then
				prx10[counter] = 1
			elseif paraT.proxTR[10] == 0 then
				prx10[counter] = 0
			end	
			if paraT.proxTR[4] ~= 0 then
				prx4[counter] = 1
			elseif paraT.proxTR[4] == 0 then
				prx4[counter] = 0
			end
			iCount10 = 0
			if (paraT.proxTR[10]>0 and #prx10>30) then
				for i=1, 30 do 
					if prx10[#prx10-i] == 0 then 
						iCount10 = iCount10 + 1
					end
				end
			end
			iCount4 = 0
			if (paraT.proxTR[4]>0 and #prx4>30) then
				for i=1, 30 do 
					if prx4[#prx4-i] == 0 then 
						iCount4 = iCount4 + 1
					end
				end
			end
			if iCount4 == 30 then
				vns.countFlag4 = 10
			elseif iCount10 == 30 then
				vns.countFlag10 = 10
			end	
		end
		-- listen to drive from parent
		for _, msgM in pairs(vns.Msg.getAM(vns.parentS, "reverseMove")) do
			vns.reverseMove = msgM.dataT	-- set the direction of the movement (forward or backward) based on the "reverseMove" message received from parent
		end
		local chillRate = 0.1
		self.lastReceivedSpeed.locV3 = self.lastReceivedSpeed.locV3 * chillRate
		self.lastReceivedSpeed.dirV3 = self.lastReceivedSpeed.dirV3 * chillRate
		for _, msgM in pairs(vns.Msg.getAM(vns.parentS, "drive")) do	-- the movement instructions are received through this message (this message is sent by each parent at each step)
			-- a drive message data is:
			--	{	yourLocV3, yourDirQ,
			--		transV3, rotateV3
			--	}			
			msgM.dataT.transV3 = vns.Msg.recoverV3(msgM.dataT.transV3)
			msgM.dataT.rotateV3 = vns.Msg.recoverV3(msgM.dataT.rotateV3)
			msgM.dataT.yourLocV3 = vns.Msg.recoverV3(msgM.dataT.yourLocV3)
			msgM.dataT.yourDirQ = vns.Msg.recoverQ(msgM.dataT.yourDirQ)

			local transV3 = Linar.mySpeedToYou(
				msgM.dataT.transV3,
				msgM.dataT.yourDirQ
			)
			local rotateV3 = Linar.mySpeedToYou(
				msgM.dataT.rotateV3,
				msgM.dataT.yourDirQ
			)

			self.lastReceivedSpeed.locV3 = transV3
			self.lastReceivedSpeed.dirV3 = rotateV3 
		end

		local transV3 = self.lastReceivedSpeed.locV3
		local rotateV3 = self.lastReceivedSpeed.dirV3

		-- add random speed
		-- this speed makes the brain move forward (if vns.reverseMove is equal to 0) or backward (if vns.reverseMove is equal to 1) 
		-- also, according to this speed an independent ground robot can move straightforward
		if vns.randomWalkerSpeed ~= nil then
			local scalar = 1
			transV3 = transV3 + vns.randomWalkerSpeed.locV3 * scalar
			rotateV3 = rotateV3 + vns.randomWalkerSpeed.dirV3 * scalar
			randomflag = 1
		end

		-- if a ground robot receives adjust_left message, it sets adjust_L to 1
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "adjust_left")) do
			adjust_L = 1
		end
		-- if a ground robot receives adjust_right message, it sets adjust_R to 1
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "adjust_right")) do
			adjust_R = 1
		end
		-- if a ground robot receives normal message, it sets its normal variable to 1; as already mentioned, this flag is used for obstacle avoidance when the MNS is shifting to left or right
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "normal")) do
			vns.normal = 1
		end
		-- if a ground robot receives avoid message, it sets flagAvoid to 1; as already mentioned, this falg is used to control robot-robot collisions when they are avoiding obstacles
		-- when this flag is 1, the robot stops moveing if it senses an object and just changes its direction depending on some conditions; in this way, the other robot can easily avoid the stationary robot
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "avoid")) do
			flagAvoid = 1
		end

	        -- lines 184 to 197 allows the robot to frequently adjust its y coordiate with respect to its y coordiate of the target point in the formation to stay approximately in the middle of its lane in the formation
	        -- the first 1000 steps is used to form the MNS; The MNS starts moving at step 1001 and the experimental data is recorded onward
		-- if the l_timer is 0, none of proximity sensors of the front half of the ground robot senses an obstacle, the time step is greater than 1000, and adjust_L is 1, the robot gradually moves forward slightly toward its left
		-- after one step, the l_timer is set to 4, and for the 4 following steps the robot does not perfrom this motion
		if l_timer == 0 and vns.robotType == "vehicle" and sum == 0 and paraT.stepCounter > 1000 and adjust_L == 1 then
					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(1.2,1.2,0),	-- left wheel's speed: 3, and right wheel's speed: 10
					}
					l_timer = 4	-- the l_timer is set to 4
					transV3 = vns.boxAvoiderSpeed.locV3
		-- if the r_timer is 0, none of proximity sensors of the front half of the ground robot senses an obstacle, the time step is greater than 1000, and adjust_R is 1, the robot gradually moves forward slightly toward its right
		-- after one step, the r_timer is set to 3, and for the 3 following steps the robot does not perfrom this motion					
		elseif r_timer == 0 and vns.robotType == "vehicle" and sum == 0 and paraT.stepCounter > 1000 and adjust_R == 1 then
					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(1.2,-1.2,0), -- left wheel's speed: 10, and right wheel's speed: 3
					}
					r_timer = 3    -- the r_timer is set to 3
					transV3 = vns.boxAvoiderSpeed.locV3
		elseif vns.robotType == "vehicle" and sum~=0 then -- if the ground robot is sensing an object
			local scalar = 1.2
	                -- as mentioned before, when a ground robot receives avoid message, it sets flagAvoid variable to 1 (see line 176)
			-- when this flagAvoid is 1, the robot stops moving and just changes its direction depending on some conditions			
			if vns.normal == 0 and paraT.stepCounter > 1000 and flagAvoid == 1 then
				if paraT.proxTR[12]~=0  then
					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(0,1,0),	-- rotate to left on spot (left wheel's speed: -7, and right wheel's speed: 7)
					}
					transV3 = vns.boxAvoiderSpeed.locV3
				elseif paraT.proxTR[2]~=0  then
					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(0,-1,0),	-- rotate to right on spot (left wheel's speed: 7, and right wheel's speed: -7)
					}
					transV3 = vns.boxAvoiderSpeed.locV3
				else
					vns.boxAvoiderSpeed = {
						locV3 = Vec3:create(0,0,0),	-- no movement
					}
					transV3 = vns.boxAvoiderSpeed.locV3
				end
								
			elseif vns.boxAvoiderSpeed ~= nil then
				transV3 = vns.boxAvoiderSpeed.locV3	-- this speed is calculated by the boxAvoider module (i.e., boxAvoider.lua) for avoiding obstacles
			else
				transV3 = transV3
			end

		end
		-- lines 231 to 251 simulate a boustrophedon path for the MNS:
			-- the brain UAV stops for 200 steps as soon as detecting a border in its front or back (i.e., after sweeping a lane) to let the ground robots that are behind reach the group
			-- then, the brain for 714 time steps shifts; afterwards, it stays stationary for 100 steps
			-- next, the brain reverses the direction of its movement: from forward (i.e., vns.reverseMove == 0) to backward (i.e., vns.reverseMove == 1) or from backward to forward
		if vns.robotType == "quadcopter"  and my_Delay == 0 then
			if paraT.cornerN ~= 0 and Mytimer == 0 then
				Mytimer = 1014
			end
			if Mytimer > 814 then	-- stay stationary for 200 steps when a border s detected
				transV3 = Vec3:create(0,0,0)
			end
			if Mytimer > 100 and Mytimer <= 814 then	-- shift along the border for 2 meters (for 714 steps) with speed 7 cm/s
				for _, robotVns3 in pairs(paraT.vehiclesTR) do
					vns.Msg.send(robotVns3.idS, "normal")
				end
				vns.speed(0.0, 0.07, 0.0)
				flag = 1
				lock = 1						
			end
			if Mytimer <= 100 and Mytimer > 0 then	-- stay stationary for 100 steps before reversing the doirection of sweep
				vns.speed(0.0, 0.0, 0.0)
				flag = 1
				lock = 1
			end
		end

		if flag == 0 and lock == 0 then	-- the following function is called by the brain in order to move forward or backward (depending on the movement direction); it is also called by non-brain UAVs and ground robots in order to move
			vns.move(transV3, rotateV3, paraT.stepCounter, randomflag)	
		end
		-- send drive to children
		for _, robotVns in pairs(vns.childrenTVns) do -- for all children
			if robotVns.rallyPoint == nil then 
				robotVns.rallyPoint = {
					locV3 = Vec3:create(),
					dirQ = Quaternion:create(),
				}
			end

			-- calc speed
			local totalTransV3, totalRotateV3

			-- add rallypointspeed
			local rallypointScalar = 2
			local dV3 = robotVns.rallyPoint.locV3 - robotVns.locV3
			local d = dV3:len()
			
			local dV3Temp = robotVns.rallyPoint.locV3 - robotVns.locV3

			-- executed by brain: define a temporary  x coordinate for the target point that is closer to the ground robot compared to the x coordinate of the actual target point: to help the robot get back to the approximately middle of its lane faster if it is not there
			if vns.idS == vns.brainS and robotVns.robotType == "vehicle" and math.abs(robotVns.rallyPoint.locV3.y - robotVns.locV3.y) > 1  then
				if reverseMove == 0 then 
					dV3Temp.x = dV3Temp.x-0.5
				else
					dV3Temp.x = dV3Temp.x+0.5
				end 
				
				local dTemp = dV3Temp:len()
				rallypointTransV3 =  rallypointScalar *dTemp * dV3Temp:nor()
			-- executed by brain: use the actual rallypoint used in the default MNS project
			elseif vns.idS == vns.brainS and robotVns.robotType == "vehicle" and math.abs(robotVns.rallyPoint.locV3.y - robotVns.locV3.y) <= 2  then		
				rallypointTransV3 = rallypointScalar / d * dV3:nor()
			else	-- for UAVs:  use the actual rallypoint used in the default MNS project			
				rallypointTransV3 = rallypointScalar / d * dV3:nor()
			end


			local rotateQ = robotVns.rallyPoint.dirQ * robotVns.dirQ:inv()
			local ang = rotateQ:getAng()
			if ang > math.pi then ang = ang - math.pi * 2 end
			local rallypointRotateV3 = rotateQ:getAxis() * ang

			-- the ground robots are not asked to move when their distance from the target point is less than 2 cm (in the view sight of the parent UAV)
			if robotVns.robotType == "vehicle" and d < 2 then rallypointTransV3 = Vec3:create() end
			-- the non-brain UAVs are not asked to move when their distance from the target point is less than 1 cm (in the view sight of the parent UAV) 
			if robotVns.robotType == "quadcopter" and d < 1 then rallypointTransV3 = Vec3:create() end
			-- the ground robots are not asked to change their orientation when their orientation with respect to the target point is less than math.pi/12 (in the sight view of the parent UAV)
			if  robotVns.robotType == "vehicle" and rallypointRotateV3:len() < math.pi/12 then rallypointRotateV3 = Vec3:create() end
			-- the non-brain UAVs are not asked to change their orientation
			if robotVns.robotType == "quadcopter"  then rallypointRotateV3 = Vec3:create() end

			-- if the distance between a ground robot and its target point is greater than 5 (in the view sight of the parent UAV), the parent UAV adjusts the y coordinate of the robot based on the current brain's movement direction and the difference between y coordinate of the robot and that of its corresponding target point
			-- this is done through sending "adjust_left" and "adjust_right" messages to the corresponding ground robot
			-- by adjusting the y coordinate of a ground robot, it gets back to the approximately middle of its lane in the formation
			if robotVns.robotType == "vehicle" and d > 5 and (Mytimer <= 100 or Mytimer > 814) then 
				if ((robotVns.rallyPoint.locV3.y - robotVns.locV3.y > 5 and vns.reverseMove == 0) or (robotVns.rallyPoint.locV3.y - robotVns.locV3.y < -5 and vns.reverseMove == 1)) then
					vns.Msg.send(robotVns.idS, "adjust_left")			
				elseif ((robotVns.rallyPoint.locV3.y - robotVns.locV3.y < -5 and vns.reverseMove == 0) or (robotVns.rallyPoint.locV3.y - robotVns.locV3.y > 5 and vns.reverseMove == 1)) then
					vns.Msg.send(robotVns.idS, "adjust_right")			
				end
	 		end


			totalTransV3 = rallypointTransV3
			totalRotateV3 = rallypointRotateV3

			local timestep = 1 / 50
			-- add parent speed
			local parentScalar = 0
			totalTransV3 = totalTransV3 + (transV3+rotateV3*robotVns.locV3) * timestep * parentScalar
			totalRotateV3 = totalRotateV3 + rotateV3 * timestep * parentScalar

			-- the following "for" loop is used for robot-robot collision avoidance (while they are avoiding obstacles) which simulates a traffic rule strategy by a parent UAV
			-- based on the rule, when two ground robots get closer than 3 cm to each other, depending on the movement direction of the brain, the robot that is behind of the other one stops moving temporarily (but might change its direction based on some conditions; lines 202 to 218), and the other one avoids it
			-- to this end, a "avoid" message is sent to the robot that is located behind and a "avoid2" message is sent to the other one (avoid2 is used by the boxAvoider module)
			for _, robotVns2 in pairs(paraT.vehiclesTR) do	-- for all the ground robots that can be seen
				if robotVns.idS ~= robotVns2.idS then
					relativeV = robotVns.locV3 - robotVns2.locV3
					local disTo = math.sqrt(relativeV.x * relativeV.x + relativeV.y * relativeV.y )
					
					if disTo < 24 then   -- this is equivalent to 3 cm distance in the simulation in practice
						if (vns.reverseMove == 0) then
							if vns.idS == vns.brainS then
								if robotVns2.locV3.x > robotVns.locV3.x then
									vns.Msg.send(robotVns.idS, "avoid")
									vns.Msg.send(robotVns2.idS, "avoid2")
								end
							elseif vns.idS ~= vns.brainS then
								if robotVns2.locV3.x > robotVns.locV3.x then
									vns.Msg.send(robotVns.idS, "avoid")
									vns.Msg.send(robotVns2.idS, "avoid2")
								end
								
							end
						elseif (vns.reverseMove == 1) then
							if vns.idS == vns.brainS then
								if vns.idS == vns.brainS then
									if robotVns2.locV3.x < robotVns.locV3.x then
										vns.Msg.send(robotVns.idS, "avoid")
										vns.Msg.send(robotVns2.idS, "avoid2")
									end
								elseif vns.idS ~= vns.brainS then
									if robotVns2.locV3.x < robotVns.locV3.x then
										vns.Msg.send(robotVns.idS, "avoid")
										vns.Msg.send(robotVns2.idS, "avoid2")
									end

								end
							end 
						end
						
					end
					
				end
			end
			-- send drive cmd
			vns.Msg.send(robotVns.idS, "drive",
				{	yourLocV3 = robotVns.locV3,
					yourDirQ = robotVns.dirQ,
					transV3 = totalTransV3:nor(),
					rotateV3 = totalRotateV3:nor(),
				}
			)
		end
end

return Driver

