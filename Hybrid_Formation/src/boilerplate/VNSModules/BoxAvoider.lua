-- External Avoider --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local BoxAvoider = {VNSMODULECLASS = true}
BoxAvoider.__index = BoxAvoider
timer = 0
timerFlag = 0
randomNum = 0
randomNumCount = 0
local flag_1 = 1
local counter_flag_1 = 0
function BoxAvoider:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function BoxAvoider:run(vns, paraT)
	local flagAvoid = 0
	if vns.boxAvoiderSpeed == nil then
		vns.boxAvoiderSpeed = {
			locV3 = Vec3:create(),
			dirV3 = Vec3:create(),
		}
	end
	-- if a ground robot receives "normal" message, it sets its normal variable to 1; this variable is used for obstacle avoidance when the MNS is shifting to left or right
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "normal")) do
		vns.normal = 1
	end
	-- to avoid robot-robot collisions during obstacle avoidance (when the MNS is moving forward or backward) a traffic rule strategy is applied by a parent UAV
	-- based on the rule, when two ground robots get closer than 3 cm to each other, depending on the movement direction of the MNS, the robot that is more "behind" stops moving temporarily (but might change its direction based on some conditions; lines 202 to 218 of Driver.lua), and the other one avoids it
	-- to this end, a "avoid" message is sent to the robot that is located behind (this message is used by Driver.lua) and a "avoid2" message is sent to the other one
	-- when a ground robot receives "avoid2" message, it sets flagAvoid to 1
	for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "avoid2")) do
		flagAvoid = 1
	end
	-- the following timer is used in the intitalization phase
	if timer > 0 then
		timer = timer - 1
		if timer == 0 then
			timerFlag = 0
		end
	end
	-- reduce the timer of the chosen turn direction (useable for cases in which only proximity sensor 1 senses an obstacle) - if the ground robot has a paren UAV
	if randomNumCount > 0 then
		randomNumCount = randomNumCount - 1
	end 
	-- reduce the timer of the chosen turn direction (useable for cases in which only proximity sensor 1 senses an obstacle) - if the ground robot is does not have any parent UAV (i.e., independent)
	if counter_flag_1 > 0 then
		counter_flag_1 = counter_flag_1 - 1
	end
	
	if  vns.parentS == nil then	-- if the robot does not have any parent UAV
		if paraT.proxTR[1]~=0 then
			if counter_flag_1 == 0 then
				local random_number = math.random()	-- a random real number between 0 and 1. This number is used to select the direction of turn (50% right and 50% left) when only proximity sensor 1 senses an obstacle.
				if random_number < 0.5 then	-- select the speed and direction of turn (50% right and 50% left)
					flag_1 = 1
				else
					flag_1 = -1
				end
				counter_flag_1 = 20 -- randomNumCount (a counter) is set to 20: for 20 steps the direction of turn is not changed as long as only proximity sensor 1 senses an obstacle 
			end
			if paraT.proxTR[1]<0.99 and flag_1 == 1 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0, -1, 0)	    -- left wheel's speed: 7, and right wheel's speed: -7
			elseif paraT.proxTR[1]<0.99 and flag_1 == -1 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)	    -- left wheel's speed: -7, and right wheel's speed: 7
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0)	    -- left wheel's speed: -7, and right wheel's speed: -7
			end
		elseif paraT.proxTR[2]~=0 then
			if paraT.proxTR[2]<0.99 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -1, 0)	    -- left wheel's speed: 7, and right wheel's speed: 0
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0)	    -- left wheel's speed: -7, and right wheel's speed: -7
			end
		elseif paraT.proxTR[3]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -1, 0)	    -- left wheel's speed: 7, and right wheel's speed: -7
		elseif paraT.proxTR[12]~=0 then	
			if paraT.proxTR[12]<0.99 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 1, 0)	    -- left wheel's speed: 0, and right wheel's speed: 7
			else
				vns.boxAvoiderSpeed.locV3 = Vec3:create(-1, 0, 0)	    -- left wheel's speed: -7, and right wheel's speed: -7
			end
		elseif paraT.proxTR[11]~=0 then
			vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 1, 0)	    -- left wheel's speed: -7, and right wheel's speed: 7
		else
			vns.boxAvoiderSpeed = nil
		end
	else
		-- when the MNS is shifting to left or right (i.e., vns.normal==1), or after initialziation phase, when the MNS is moving forward or backward but no robot-robot collision is observed by the UAVs (i.e., flagAvoid == 0 and  paraT.stepCounter >= 1000), the following set of static rules for obstacle avoidance is used by the ground robots
		if vns.normal == 1 or (flagAvoid == 0 and  paraT.stepCounter >= 1000) then
			if paraT.proxTR[12]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.2, 0.2, 0)	    -- left wheel's speed: -7, and right wheel's speed: 10
				-- the following flag is used by the robot (structure_vehicle.lua; by "move" function which is the function that is called after applying all the speeds and is responsible for the movement of the robot) when it receives negative values for both left and right speeds from its parent UAV 
				-- When flagTurn is 0, the ground robot does not sense any obstacle by its set of front half proximity sensors (i.e., proximity sensors 1, 2, 3, 4, 12, 11, 10), and the parent UAV sends negative values to the ground robots as its left wheel and right wheel speeds, the ground robot turns left on spot in that step
				-- When flagTurn is 1, the ground robot does not sense any obstacle by its set of front half proximity sensors (i.e., proximity sensors 1, 2, 3, 4, 12, 11, 10), and the parent UAV sends negative values to the ground robots as its left wheel and right wheel speeds, the ground robot turns right on spot in that step
				vns.flagTurn = 1
			elseif paraT.proxTR[2]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.2, -0.2, 0)	    -- left wheel's speed: 10, and right wheel's speed: -7
				vns.flagTurn = 0
			elseif paraT.proxTR[3]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.2, -0.08, 0)	    -- left wheel's speed: 7, and right wheel's speed: -1
				vns.flagTurn = 0
			elseif paraT.proxTR[4]~=0 then    -- this rule simulates a half-circle trajectory from right side of an obstacle	
				if vns.countFlag4 == 0 or vns.countFlag4 == nil then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.02, 0)	    -- left wheel's speed: 8.1, and right wheel's speed: 6.9
				else 
					vns.countFlag4 = vns.countFlag4 - 1
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.1, 0)	    -- left wheel's speed: 4.5, and right wheel's speed: 10
				end
				vns.flagTurn = 0
			elseif paraT.proxTR[11]~=0 then	
				if paraT.proxTR[11]<0.99 then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.2, 0.08, 0)	    -- left wheel's speed: -1, and right wheel's speed: 7
				else
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)	    -- left wheel's speed: -10, and right wheel's speed: 10
				end
				vns.flagTurn = 1
			elseif paraT.proxTR[10]~=0 then    -- this rule simulates a half-circle trajectory from left side of an obstacle
				if vns.countFlag10 == 0 or vns.countFlag10 == nil then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.02, 0)	    -- left wheel's speed: 6.9, and right wheel's speed: 8.1
				else 		
					vns.countFlag10 = vns.countFlag10 - 1	
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.1, 0)	    -- left wheel's speed: 10, and right wheel's speed: 4.5
				end
				vns.flagTurn = 1
			elseif paraT.proxTR[1] ~= 0 then
				if randomNumCount == 0 then
					randomNum = math.random()   -- a random real number between 0 and 1. This number is used to select the direction of turn (50% right and 50% left) when only proximity sensor 1 senses an obstacle
					randomNumCount = 20   -- randomNumCount (a counter) is set to 20: for 20 steps the direction of turn is not changed as long as only proximity sensor 1 senses an obstacle
				end
				if randomNum > 0.5 then   -- select the speed and direction of turn (50% right and 50% left)	
					vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.8, 0)	    -- left wheel's speed: 10, and right wheel's speed: 3
					vns.flagTurn = 0
				else
					vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.8, 0)	    -- left wheel's speed: 3, and right wheel's speed: 10
					vns.flagTurn = 1
				end

			end
		-- when a ground robot receives "avoid2" message, the following set of static rules for obstacle avoidance is used by the ground robot
		elseif flagAvoid == 1 and paraT.stepCounter>1000 then
			if paraT.proxTR[4]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.8, 0)	    -- left wheel's speed: 10, and right wheel's speed: 3	
			elseif paraT.proxTR[10]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.8, 0)	    -- left wheel's speed: 3, and right wheel's speed: 10
			elseif paraT.proxTR[12]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0)	    -- left wheel's speed: -2.8, and right wheel's speed: 5.2		
				vns.flagTurn = 1
			elseif paraT.proxTR[2]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)	    -- left wheel's speed: 5.2, and right wheel's speed: -2.8
				vns.flagTurn = 0
			elseif paraT.proxTR[3]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0)	    -- left wheel's speed: 5.5, and right wheel's speed: -2.5
				vns.flagTurn = 0		
			elseif paraT.proxTR[11]~=0 then	
				if paraT.proxTR[11]<0.99 then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0)	    -- left wheel's speed: -2.5, and right wheel's speed: 5.5
				else
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)	    -- left wheel's speed: -10, and right wheel's speed: 10
				end
				vns.flagTurn = 1
			end
		-- the following set of static rules are used for obstacle avoidance for the initialization phase (i.e., the first 1000 steps)	
		elseif  paraT.stepCounter<1000 then
			if paraT.proxTR[12]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, 0.08, 0)	-- left wheel's speed: -2.8, and right wheel's speed: 5.2			
				vns.flagTurn = 1
			elseif paraT.proxTR[2]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)	-- left wheel's speed: 5.2, and right wheel's speed: -2.8			
				vns.flagTurn = 0
			elseif paraT.proxTR[1]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.08, -0.08, 0)	-- left wheel's speed: 5.2, and right wheel's speed: -2.8
				vns.flagTurn = 0
			elseif paraT.proxTR[3]~=0 then
				vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, -0.08, 0)	-- left wheel's speed: 5.5, and right wheel's speed: -2.5
				vns.flagTurn = 0
			elseif paraT.proxTR[4]~=0 then		
				if vns.countFlag4 == 0 or vns.countFlag4 == nil then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(1, -0.2, 0)	-- left wheel's speed: 10, and right wheel's speed: 10
				else 			
					vns.countFlag4 = vns.countFlag4 - 1
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, 0.1, 0)	-- left wheel's speed: 4.5, and right wheel's speed: 10
				end
				vns.flagTurn = 0
			elseif paraT.proxTR[11]~=0 then
				if paraT.proxTR[11]<0.99 then
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0.1, 0.08, 0)	-- left wheel's speed: -2.5, and right wheel's speed: 5.5
				else
					vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 1, 0)	-- left wheel's speed: -10, and right wheel's speed: 10
				end
				vns.flagTurn = 1
			elseif paraT.proxTR[10]~=0 and paraT.stepCounter > 980 then
				if timerFlag == 0 then
					timer = 20	-- set timer to 20
					timerFlag = 1
				end
				if timer >= 10 then	-- if timer is equal or greater than 10, do not move or rotate for 10 steps
					if vns.countFlag10 == 0 or vns.countFlag10 == nil then
						vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 0, 0)	-- left wheel's speed: 0, and right wheel's speed: 0
					else 		
						vns.countFlag10 = vns.countFlag10 - 1	
						vns.boxAvoiderSpeed.locV3 = Vec3:create(0, 0, 0)	-- left wheel's speed: 0, and right wheel's speed: 0
					end
				else	-- otherwise: this rule simulates a half-circle trajectory from left side of an obstacle	
					if vns.countFlag10 == 0 or vns.countFlag10 == nil then
						vns.boxAvoiderSpeed.locV3 = Vec3:create(1, 0.2, 0)	-- left wheel's speed: 10, and right wheel's speed: 10
					else 		
						vns.countFlag10 = vns.countFlag10 - 1	
						vns.boxAvoiderSpeed.locV3 = Vec3:create(0.5, -0.1, 0)	-- left wheel's speed: 10, and right wheel's speed: 4.5
					end
				end
				vns.flagTurn = 1
			end
		end
	end
end



return BoxAvoider

