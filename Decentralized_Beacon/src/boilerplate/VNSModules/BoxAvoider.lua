-- External Avoider --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")
local flag_1 = 1
local counter_flag_1 = 0

local BoxAvoider = {VNSMODULECLASS = true}
BoxAvoider.__index = BoxAvoider

function BoxAvoider:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function BoxAvoider:run(vns, paraT)
	if vns.boxAvoiderSpeed == nil then
		vns.boxAvoiderSpeed = {
			locV3 = Vec3:create(),
			dirV3 = Vec3:create(),
		}
	end
	-- reduce the timer of the chosen turn direction (useable for cases in which only proximity sensor 1 senses an obstacle)
	if counter_flag_1 > 0 then
		counter_flag_1 = counter_flag_1 - 1
	end
	if paraT.proxTR[1]~=0 then
		if counter_flag_1 == 0 then
			local random_number = math.random()	-- a random real number between 0 and 1. This number is used to select the direction of turn (50% right and 50% left) when only proximity sensor 1 senses an obstacle
			if random_number < 0.5 then
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


end



return BoxAvoider

