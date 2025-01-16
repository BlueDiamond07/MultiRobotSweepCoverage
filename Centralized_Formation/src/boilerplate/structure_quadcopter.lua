------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"

local IF = {} -- Robot Interface
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.VehicleConnector,
	VNS.Modules.QuadcopterConnector,
	VNS.Modules.LostCounter,
	VNS.Modules.Shifter,
	VNS.Modules.RandomWalker,
	VNS.Modules.Driver,
}

local vns
fn = ''
global_folder = ''
fii = 0
------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	stepCounter = 0
	----------------------------------------
	-- create a folder per each run in order to record the speed values of robots over time separately
	global_folder = 'Speeds_data/'..os.date('%d-%m-%y_%H-%M-%S')
	os.execute("mkdir " .. global_folder)
	----------------------------------------
	local dis = 20
       -- the following structrue is used in case of using 8 ground robots
	local structure = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
		children = {
			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,0.45*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,-0.45*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,1.25*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,-1.25*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
					
			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,2.1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,-2.1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,3*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(0*dis,-3*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},	
		},
	}

	vns = VNS:new{idS = IF.myIDS(), robotType = "quadcopter"}
	vns.modules[4]:setGene(vns, structure)

	reset()
end

-------------------------------------------------------------------
function reset()
	vns:reset()
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1	-- increase time step by one at each step
	if (stepCounter == 1) then
		fn=os.date(global_folder..'/%d-%m-%y_%H-%M-%S_'..IF.myIDS()..'.csv')	-- create a csv file per each robot to record its speed values over time
	end
	vns:run{vehiclesTR = getVehicleTR(), 
	        boxesTR = getBoxesTR(),
	        predatorsTR = getPredatorsTR(), cornerN = detectCorner(), stepCounter = stepCounter, myid = IF.myIDS()}

end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
function detectCorner()	-- this function is used for detecting the border from front of the formation or from back of the formation
	local cornerflag = 0
	for i, detectionT in ipairs(IF.getLEDsT()) do
		-- not used in this project: for detecting a white LED in the specified range from the supervisor UAV (supervisor's camera)
		if (vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.color.blue == 255 and ((detectionT.center.x>=281 and detectionT.center.x<=308) or (detectionT.center.x>=350 and detectionT.center.x<=360)) and detectionT.center.y < 304 and detectionT.center.y > 175) then
		-- for detecting a border (yellow LEDs) from the front of the formation, in the specified range from the supervisor UAV (supervisor's camera)			
		elseif (vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.center.x>=325 and detectionT.center.x<=337.4) then
			cornerflag = 1
		-- for detecting a border (yellow LEDs) from the back of the formation, in the specified range from the supervisor UAV (supervisor's camera)
		elseif (stepCounter > 1500 and vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.center.x>=300.5 and detectionT.center.x<=307) then
			cornerflag = 2
		end	
	end	
	return cornerflag
end

function getVehicleTR() -- this function returns the ID, relative orentation and relative position and heading of observed robots
	local vehicleTR = {}
	for i, tagDetectionT in pairs(IF.getTagsT()) do
		-- a tag detection is a complicated table
		-- containing center, corners, payloads
		
		-- get robot info
		local locV3, dirQ, idS, front= getVehicleInfo(tagDetectionT)
			-- locV3 for vector3 {x,y,z}
			-- loc (0,0) in the middle, x+ right, y+ up , 
			-- dir a quaternion, x+ as 0

		vehicleTR[idS] = {locV3 = locV3, dirQ = dirQ, idS = idS, front = front}
	end
	return vehicleTR
end

function getVehicleInfo(tagT) -- this function is called by getVehicleTR()
	-- a tag is a complicated table with center, corner, payload
	local degQ, front = calcVehicleDir(tagT.corners)
		-- a direction is a Quaternion
		-- with 0 as the x+ axis of the quadcopter
	front.x = front.x - 320
	front.y = -(((tagT.corners[3].y + tagT.corners[4].y) / 2)-240) 		
	local x = tagT.center.x - 320
	local y = tagT.center.y - 240
	y = -y 				-- make it left handed coordination system
	local z = 0
	local locV3 = Vec3:create(x,y,z)

	local idS = tagT.payload

	return locV3, degQ, idS, front
end

function calcVehicleDir(corners)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local front = {}
	front.x = (corners[3].x + corners[4].x) / 2
	front.y = -(corners[3].y + corners[4].y) / 2
	local back = {}
	back.x = (corners[1].x + corners[2].x) / 2
	back.y = -(corners[1].y + corners[2].y) / 2
	local deg = calcDir(back, front)
	return deg, front
end

function calcDir(center, target)
	-- from center{x,y} to target{x,y} in left hand
	-- return a Quaternion
	local x = target.x - center.x
	local y = target.y - center.y
	local rad = math.atan(y / x)
	if x < 0 then
		rad = rad + math.pi
	end
	if rad < 0 then
		rad = rad + math.pi*2
	end
	return Quaternion:create(0,0,1, rad)
end

function getBoxesTR()	-- this function is not used
	local boxesTR = {}   -- vt for vector, which a table = {x,y}
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if (ledDetectionT.color.red ~= 255 and ledDetectionT.color.green ~= 255) and (ledDetectionT.color.blue ~= 0 or ledDetectionT.color.red ~= 0 or ledDetectionT.color.green ~= 0) then
			-- a detection is a table
			-- {center = {x,y}, color = {blue, green, red}}
			local locV3, dirQ, colorS = getBoxInfo(ledDetectionT)
			boxesTR[i] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
		end
	end	
	return boxesTR
end

function getBoxInfo(detection)	-- this function is not used
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "orange"
	if (detection.color.blue == 0 and detection.color.red == 0 and detection.color.green == 255) then
		colorS = "green"
	elseif (detection.color.blue == 0 and detection.color.red == 255 and detection.color.green == 0) then
		colorS = "red"
	elseif (detection.color.blue == 255 and detection.color.red == 0 and detection.color.green == 0) then
		colorS = "blue"
	elseif (detection.color.blue > 0 and detection.color.red > 0 and detection.color.green ~= 255) then
		colorS = "purple"				
	end
	return locV3, dirQ, colorS
end

function getPredatorsTR()	-- this function is not used
	local predatorsTR = {}   -- vt for vector, which a table = {x,y}
	local j = 0
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if ledDetectionT.color.blue > 150 then -- else continue
		j = j + 1
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getPredatorInfo(ledDetectionT)
		predatorsTR[j] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	end
	return predatorsTR
end

function getPredatorInfo(detection)	-- this function is not used
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "blue"
	return locV3, dirQ, colorS
end

------------------------------------------------------------------------
--   VNS Callback Functions
------------------------------------------------------------------------
VNS.Msg.myIDS = function()
	return IF.myIDS()
end

VNS.Msg.Packet.sendData = function(dataAN)
	robot.radios["radio_0"].tx_data(dataAN)	
end

VNS.Msg.Packet.receiveDataAAN = function()
	return robot.radios["radio_0"].rx_data
end

-- this function is called only when the suoervisor UAV is moving forward or backward
VNS.move = function(transV3, rotateV3, time)
	local speedscale = 0.14
	local turnscale = 1
	local x = transV3.x * speedscale
	local y = transV3.y * speedscale
	local w = rotateV3:len() * turnscale
	if rotateV3.z < 0 then w = -w end
	-- depending on the movement direction, the speed is equal to either 7 cm/s or -7 cm/s (i.e., either x = 0.07 or x = -0.07) when this function is called
	IF.setVelocity(x, y, w)
end

-- this function is called during the time that the supervisor UAV is shifting or during the time that the supervisor UAV is waiting after finishing a shift process along a border
VNS.speed = function(x, y, w)
	IF.setVelocity(x, y, w)
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.myIDS()
	return robot.id
end

function IF.getTagsT()
	return robot.cameras.fixed_camera.tag_detector
end

function IF.getLEDsT()
	return robot.cameras.fixed_camera.led_detector
end

function IF.setVelocity(x, y, w)
	-- the following is not necessary as the supervisor UAV definitely stays stationary for the last 200 steps as it detects the boundary:
	if stepCounter >= 5090 then
		x = 0
		y = 0
	end
	--quadcopter heading is the x+ axis
	local thRad = robot.joints.axis2_body.encoder
	local xWorld = x * math.cos(thRad) - y * math.sin(thRad)
	local yWorld = x * math.sin(thRad) + y * math.cos(thRad)
	robot.joints.axis0_axis1.set_target(xWorld)
	robot.joints.axis1_axis2.set_target(yWorld)
	robot.joints.axis2_body.set_target(w)
	----------------------------------------
	-- record the speed info of the UAV in addition to its ID and the relevant time step	
	if (stepCounter>1000) then
		fii = io.open (fn,"a" )
		fii:write(IF.myIDS().. "\t")
		fii:write((stepCounter-1000).. "\t")
		fii:write(x.. "\t")
		fii:write(y.. "\t")
		fii:write(w.. "\n")	
		fii:close()
	end
	----------------------------------------
	
end

