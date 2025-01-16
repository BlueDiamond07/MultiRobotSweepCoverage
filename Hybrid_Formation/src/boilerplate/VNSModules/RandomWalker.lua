-- Random Walker--------------------------------
------------------------------------------------------
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")
local Linar = require("Linar")

local RandomWalker = {VNSMODULECLASS = true}
RandomWalker.__index = RandomWalker

function RandomWalker:new()
	local instance = {}
	setmetatable(instance, self)
	return instance
end

function RandomWalker:run(vns, paraT)
	-- work only no parent
	if vns.parentS ~= nil then 
		vns.randomWalkerSpeed = nil
		return 
	end

	local x = math.random()
	local y = math.random() - 0.5
	local z = 0
	if vns.robotType == "quadcopter" and paraT.stepCounter<1000 then -- make the brain stay stationary in the first 1000 steps (i.e., during the initilization phase)
		x = 0
		y = 0
	elseif vns.robotType == "quadcopter" then  -- make the brain move forward
		if vns.reverseMove == 0 then
			x = 30
			y = 0
		else  -- make the brain move backward
			x = -30
			y = 0
		end
	else
		x = 30
		y = 0
	end
	local transV3 = Vec3:create(x,y,z):nor() * 0.5

	-- no random change in direction (z is not used for setting the angular speed of robots; see vns.move() in vehicle.lua)
	x = 0
	y = 0 
	z = math.random() - 0.5
	if vns.robotType == "quadcopter" then
		z = 0
	end
	local rotateV3 = Vec3:create(x,y,z):nor() * 0.5

	vns.randomWalkerSpeed = {
		locV3 = transV3,
		dirV3 = rotateV3,
	}
end

return RandomWalker
