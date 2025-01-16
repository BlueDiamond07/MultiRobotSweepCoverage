------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.EnableModules = {

	VNS.Modules.RandomWalker,
	VNS.Modules.BoxAvoider,

	VNS.Modules.Driver,
	
}

local vns
fn = ''
global_folder = ''
fii = 0
general = 0
gLeft = nil
gRight = nil
------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	----------------------------------------
	-- create a folder per each run in order to record the speed values of robots over time separately
	global_folder = 'Speeds_data/'..os.date('%d-%m-%y_%H-%M-%S')
	os.execute("mkdir " .. global_folder)
	----------------------------------------
	vns = VNS:new{id = IF.myIDS(), robotType = "vehicle"}
	reset()

	stepCounter = 0

end

-------------------------------------------------------------------
function reset()
	IF.setTag(IF.myIDS())
	vns:reset()
end

-------------------------------------------------------------------
function step()
	if (general > 0) then
		general = general - 1
	end
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1
	if (stepCounter == 1) then
		fn=os.date(global_folder..'/%d-%m-%y_%H-%M-%S_'..IF.myIDS()..'.csv') -- create a csv file per each robot to record its speed values over time
	end
	vns:run{proxTR=IF.getProximityTableTR(),stepCounter = stepCounter, myid = IF.myIDS()}

end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
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

VNS.move = function(transV3, rotateV3,stepCounter, randomflag)
	local left = transV3.x	-- take x of transV3 (a 3D vector used to indicate motion control)  
	local right = transV3.x	-- take x of transV3 (a 3D vector used to indicate motion control)  

	local turnRate = 1
	local speedscale = 15
	if randomflag ~= nil then
		speedscale = 20
	end
	local smalllimit = 0.3	-- used to set a minimum magnitude of the vector
	-- ensure the vector has a large enough magnitude for the robot	
	if 0 <= transV3.x and transV3.x < smalllimit then transV3.x = smalllimit end
	if 0 > transV3.x and transV3.x >-smalllimit then transV3.x =-smalllimit end

	-- define left and right
	left  = left  - transV3.y/transV3.x * turnRate	 -- set as: x - (y/x)
	right = right + transV3.y/transV3.x * turnRate	 -- set as: x + (y/x)

	-- lines 101 to 112 are for reading the id of the faulty robots
	---------------------------------------------------------------
	local fault_flag = 0
	local faults = io.open ("FaultTolerance/faulty_robots.csv","r" )
	lines = faults:lines()

	if stepCounter >= 500 then
		for line in lines do
			local ID = 'vehicle'..line
			if (IF.myIDS() == ID) then
				fault_flag = 1
			end
		end
	end
	---------------------------------------------------------------
	if (fault_flag == 1) then	-- if the robot is faulty, set the speed to 0   
		IF.setVelocity(0, 0)
	else			
		left = left * speedscale
		right = right * speedscale
		-- possible outcomes are: 
			-- left wheel pos, right wheel neg (i.e., robot turns right at a constant radial velocity of 7 cm/s)
			-- left wheel neg, right wheel pos (i.e., robot turns left at a constant radial velocity of 7 cm/s)
			-- left wheel pos, right wheel pos (i.e., robot move forward at a constant linear velocity of 7 cm/s)
		-- note: in this project if the robot is not faulty, the absolute value of left and right speeds is always greater than 7 based on the parameters we set
		if left > 7 then	-- if (x - (y/x)) is higher than 7, then set the left wheel to turn forward at the constant radial velocity (7 cm/s)
			left = 7
		end
		if right > 7 then	-- if (x + (y/x)) is higher than 7, then set the right wheel to turn forward at the constant radial velocity (7 cm/s)
			right = 7
		end
		if left < -7 then	-- if (x - (y/x)) is less than -7, then set the left wheel to turn backward at the constant radial velocity (7 cm/s)
			left = -7
		end
		if right < -7 then	-- if (x + (y/x)) is less than -7, then set the right wheel to turn backward at the constant radial velocity (7 cm/s)
			right = -7
		end

		IF.setVelocity(left, right)
		
	end
end


VNS.setSpeed = function(x, y) -- this funtion is called for turning purpose as a response to the message the robot receives from a UAV
		-- lines 145 to 156 are for reading the id of the faulty robots
	local fault_flag = 0
	local faults = io.open ("FaultTolerance/faulty_robots.csv","r" )
	lines = faults:lines()

	if stepCounter >= 500 then
		for line in lines do
			local ID = 'vehicle'..line
			if (IF.myIDS() == ID) then
				fault_flag = 1
			end
		end
	end

	if (fault_flag == 1) then 
		IF.setVelocity(0, 0) -- the speed of a faulty robot is always 0
	else
		IF.setVelocity(x, y)
	end
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.getProximityTableTR()
	return robot.proximity
end

function IF.getProximityN(x)
	return robot.proximity[x]
end

function IF.myIDS()
	return robot.id
end

function IF.setTag(str)
	robot.tags.set_all_payloads(str)
end

function IF.setVelocity(x, y)
	robot.joints.base_wheel_left.set_target(x)
	robot.joints.base_wheel_right.set_target(-y)
	----------------------------------------
	-- record the left wheel speed and right wheel speed of the robot in addition to the robots ID and the relevant time step 
	fii = io.open (fn,"a" )
	fii:write(IF.myIDS().. "\t")
	fii:write((stepCounter).. "\t")
	fii:write(x.. "\t")
	fii:write(y.. "\n")	
	fii:close()
	----------------------------------------
end
