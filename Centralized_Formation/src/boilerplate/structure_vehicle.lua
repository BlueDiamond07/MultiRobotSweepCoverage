------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.ParentWaitorDeny,
	VNS.Modules.LostCounter,
	VNS.Modules.Shifter,
	VNS.Modules.RandomWalker,
	VNS.Modules.BoxAvoider,
	VNS.Modules.Driver,
}

local vns
local flagI = 0
fn = ''
global_folder = ''
fii = 0
general = 0
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

------------------------------------------------------------------------
-- Handling of all motion vectors	
------------------------------------------------------------------------
VNS.move = function(transV3, rotateV3,stepCounter, randomflag)
	-- a flag (which is set at line 130) used to manage the rotation of the robot when it receives two negative values from the UAV as its target left wheel speed and right wheel speed	
	if flagI>0 then
		flagI = flagI - 1
	end

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
	left  = left  - transV3.y/transV3.x * turnRate
	right = right + transV3.y/transV3.x * turnRate
	
	-- lines 102 to 113 are for reading the id of the faulty robots
	---------------------------------------------------------------	
	local fault_flag = 0
	local faults = io.open ("FaultTolerance/faulty_robots.csv","r" )
	lines = faults:lines()

	if stepCounter >= 1500 then
		for line in lines do
			local ID = 'vehicle'..line
			if (IF.myIDS() == ID) then
				fault_flag = 1
			end
		end
	end
	---------------------------------------------------------------
	if (fault_flag == 1) then	-- if a robot is not connected to a UAV 
		IF.setVelocity(0, 0)	-- if the robot is faulty, set the speed to 0
	else
		-- we do not let the ground robots move backward while the supervisor UAV is moveing
		-- to this end, when both left wheel and right wheel speeds sent by the supervisor UAV are negative, the ground robot rotates for 7 steps to adjust its heading with respect to the heading of the parent UAV
		-- after performing this action, the robot is not received negative speeds for both wheels concurrently
		if flagI > 0 then
			if vns.flagTurn == 0 then
				IF.setVelocity(-10, 10)
			elseif vns.flagTurn == 1 then
				IF.setVelocity(10, -10)
			end
		else
			if right < 0 and left < 0 then
				if stepCounter > 1000 then
					flagI = 7
					if vns.flagTurn == 0 then
						IF.setVelocity(-3, 10)
					elseif vns.flagTurn == 1 then
						IF.setVelocity(10, -3)
					end
				else
					-- this action is performed in initialization phase (when time step is less than 1000): 
					-- the ground robot rotates to adjust its heading with respect to the heading of the supervisor UAV					
					if vns.flagTurn == 0 then
						IF.setVelocity(-40, 40)
					elseif vns.flagTurn == 1 then
						IF.setVelocity(40, -40)
					end
				end
			else
				left = left * speedscale
				right = right * speedscale
				-- if a wheel speed is higher than 10, it is set to 10
				-- if a wheel speed is less than -10, it is set to -10 	
				-- maximum speed of the ground robot is 10 cm/s				
				if left > 10 then
					left = 10
				end
				if right > 10 then
					right = 10
				end
				if left < -10 then
					left = -10
				end
				if right < -10 then
					right = -10
				end
				IF.setVelocity(left, right)
			end
		end
	end
end


VNS.setSpeed = function(x, y)
	IF.setVelocity(x, -y)
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
	if (stepCounter>1000) then
		fii = io.open (fn,"a" )
		fii:write(IF.myIDS().. "\t")
		fii:write((stepCounter-1000).. "\t")
		fii:write(x.. "\t")
		fii:write(y.. "\n")	
		fii:close()
	end
	----------------------------------------
end
