-- Driver --------------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local Driver = {VNSMODULECLASS = true}
Driver.__index = Driver
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
		print('vns.idS: ',paraT.myid)
		local randomflag

		-- add random speed
		if vns.randomWalkerSpeed ~= nil then
			local scalar = 1
			transV3 = vns.randomWalkerSpeed.locV3 * scalar
			rotateV3 = rotateV3 + vns.randomWalkerSpeed.dirV3 * scalar
			randomflag = 1
		end
		
		local scalar = 1.2
		-- apply the box avoider speed from BoxAvoider.lua
		if vns.boxAvoiderSpeed ~= nil then
			transV3 = vns.boxAvoiderSpeed.locV3
		else
			transV3 = transV3
		end
		
		vns.move(transV3, rotateV3, paraT.stepCounter, randomflag) -- move function is called

end


return Driver

