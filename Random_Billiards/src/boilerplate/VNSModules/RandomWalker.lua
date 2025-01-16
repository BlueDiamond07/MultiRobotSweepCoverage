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

	-- make the robot move forward
	local x = 30
	local y = 0
	local z = 0

	local transV3 = Vec3:create(x,y,z):nor() * 0.5

	-- no random change in direction
	x = 0
	y = 0 
	z = 0
	local rotateV3 = Vec3:create(x,y,z):nor() * 0.5
	vns.randomWalkerSpeed = {
		locV3 = transV3,
		dirV3 = rotateV3,
	}
end

return RandomWalker
