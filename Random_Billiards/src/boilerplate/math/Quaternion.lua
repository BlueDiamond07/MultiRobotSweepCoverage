local Vec3 = require("Vector3")

local Quaternion = {CLASS = "Quaternion", CLASSQUATERNION = true}
Quaternion.__index = Quaternion

function Quaternion:createFromHardValue(x,y,z,w)
	local instance = {}
	setmetatable(instance,self)
	self.__index = self

	if type(x) == "table" and x.CLASS == "Vector3" then
		instance.v = x
		instance.w = y
		return instance
	end
	if type(x) == "number" and 
	   type(y) == "number" and
	   type(z) == "number" and
	   type(w) == "number" then
		instance.v = Vec3:create(x,y,z)
		instance.w = w
		return instance
	end

	instance.v = Vec3:create(0,0,0)
	instance.w = 0
	return instance;
end


function Quaternion:create(x,y,z,th)	-- create from rotation
	local halfth
	local v
	if type(x) == "table" and x.CLASS == "Vector3" and x:len() ~= 0 then
		halfth = y / 2
		v = x / x:len()
		v = x:nor()
		return self:createFromHardValue(v * math.sin(halfth),math.cos(halfth))
	end
	if type(x) == "number" and
	   type(y) == "number" and
	   type(z) == "number" and
	   type(th) == "number" and
	   (x ~=0 or y ~= 0 or z ~= 0)then
		halfth = th / 2
		v = Vec3:create(x,y,z)
		v = v:nor()
		return self:createFromHardValue(v * math.sin(halfth),math.cos(halfth))
	end

	return self:create(0,0,1,0)
end

function Quaternion:getAxis()
	local halfth = math.acos(self.w)
	if halfth == 0 then
		return Vec3:create(0,0,1)
	end
	return (self.v / math.sin(halfth)):nor()
end

function Quaternion:getAng()	-- in rad
	return math.acos(self.w) * 2
end

function Quaternion.__add(a,b)
	return Quaternion:createFromHardValue(a.v + b.v,a.w + b.w)
end

function Quaternion.__unm(b)
	return Quaternion:createFromHardValue(-b.v,-b.w)
end

function Quaternion.__sub(a,b)
	local c = -b
	return a + c 
end

function Quaternion.__mul(a,b)
	local tv = a.v * b.v + a.w * b.v + a.v * b.w
	local tw = a.w * b.w - a.v ^ b.v
	return Quaternion:createFromHardValue(tv,tw)
end

function Quaternion:squlen()
	return self.v:squlen() + self.w * self.w
end

function Quaternion:len()
	return math.sqrt(self:squlen())
end

--inverse
function Quaternion:inv()
	if self:len() ~= 0 then
		return Quaternion:createFromHardValue(-self.v/self:len(),self.w/self:len())
	end
	return Quaternion:createFromHardValue(0,0,0,0)
end

function Quaternion:__tostring()
	local c = "(" .. "(" .. self.v.x .. "," .. self.v.y .. "," .. self.v.z .. ")"
	c = c .. "," .. self.w .. ")"
	return c
end

-- returns a vector, is a is not a vector, returns a whatever it is
function Quaternion:toRotate(a)
	if self.v:len() == 0 and self.w == 0 then
		return a
	end
	if type(a) == "table" and a.CLASS == "Vector3" then
		local p = Quaternion:createFromHardValue(a,0)
		local res = self * p
		res = res * self:inv()
		local b = Vec3:create(res.v)
		return b
	else
		print("In Quaternion:toRotate, para not a Vector3")
		return a
	end
end

return Quaternion

