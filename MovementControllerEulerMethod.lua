--[[
============================================================
NEPTUNE-R • Physics-Based Simulation using Euler's Method
============================================================

This script implements realistic rover movement on water
without teleporting, CFrame-based motion, or Tweening.

Instead, it uses physics forces to apply a numeric solution
to the differential equations governing motion:

  F = m * a
  a = (targetVelocity - currentVelocity) / dt

This is a practical application of the **Euler Method** for
solving ordinary differential equations (ODEs):

    dy/dt = f(t, y)
    y_{n+1} = y_n + h * f(t_n, y_n)

Mapping to physics:

    v_{t+dt} = v_t + a_t * dt     -- velocity update
    p_{t+dt} = p_t + v_t * dt     -- position update

In code:

    vForce.Force = (desired - actual) * MASS / dt

Where:
  • `desired` is the target velocity toward the next waypoint
  • `actual` is the current linear velocity
  • `dt` is delta time per frame from RunService.Heartbeat

Roblox's physics engine applies the result each frame,
effectively doing:

    velocity += acceleration * dt
    position += velocity * dt

...which is the Forward Euler method in action.

This approach ensures:
  • Smooth motion
  • Realistic inertia and drag
  • No teleporting or snapping
  • Compatibility with force fields, terrain, etc.

]]


local RunService = game:GetService("RunService")
local Terrain = workspace.Terrain

local rover = workspace:WaitForChild("NEPTUNE-R")
local prt = rover.PrimaryPart
local MASS = prt:GetMass()

local SPEED = 12
local GRID = 4
local ARC_RU = 3.5
local ARC_R = ARC_RU * GRID

local GRID_WIDTH = 80 * GRID
local GRID_HEIGHT = 128 * GRID

local GRID_X_OFFSET = -200
local GRID_Z_OFFSET = 400


local ARC_TOTAL = 2 * ARC_R
local NUM_ROWS = math.floor(GRID_HEIGHT / (ARC_TOTAL + GRID))
local LINE_HEIGHT = (GRID_HEIGHT - NUM_ROWS * ARC_TOTAL) / NUM_ROWS
local TOTAL_PATTERN_HEIGHT = NUM_ROWS * (LINE_HEIGHT + ARC_TOTAL)
local Z_OFFSET = (GRID_HEIGHT - TOTAL_PATTERN_HEIGHT) / 2

local att = prt:FindFirstChildOfClass("Attachment") or Instance.new("Attachment", prt)
att.Name = "RootAttachment"

local vForce = Instance.new("VectorForce")
vForce.Attachment0 = att
vForce.RelativeTo = Enum.ActuatorRelativeTo.World
vForce.ApplyAtCenterOfMass = true
vForce.Parent = prt

local gyro = Instance.new("BodyGyro")
gyro.MaxTorque, gyro.P, gyro.D = Vector3.new(0, 1e6, 0), 5000, 500
gyro.Parent = prt

------------------------------------------------------------
--  Helpers
------------------------------------------------------------
local function mark(p: Vector3, c: Color3)
	local d = Instance.new("Part")
	d.Anchored = true
	d.CanCollide = false
	d.Size = Vector3.new(0.25, 0.25, 0.25)
	d.Position = p + Vector3.yAxis * 0.1 + Vector3.new(0, 1, 0)
	d.Color = c
	d.Material = Enum.Material.Neon
	d.Parent = workspace
end

local function isClear(pos: Vector3)
	local region = Region3.new(pos - Vector3.new(5, 4, 5), pos + Vector3.new(5, 4, 5)):ExpandToGrid(4)
	local vox = Terrain:ReadVoxels(region, 4)
	for x = 1, #vox do for y = 1, #vox[x] do for z = 1, #vox[x][y] do
				local mat = vox[x][y][z]
				if mat ~= Enum.Material.Water and mat ~= Enum.Material.Air then
					return false
				end
			end end end
	return true
end

-- Build arc from start to end with optional curve direction
local function buildArcBetween(startPos: Vector3, endPos: Vector3, steps: number, flipDirection: boolean)
	local points = {}
	local chord = endPos - startPos
	if chord.Magnitude < 1e-3 then return points end

	local dir = chord.Unit
	local mid = (startPos + endPos) / 2

	local perp = flipDirection and Vector3.new(dir.Z, 0, -dir.X) or Vector3.new(-dir.Z, 0, dir.X)
	local center = mid + perp * ARC_R
	local frame = CFrame.new(center, mid)

	for i = 0, steps do
		local t = i / steps
		local angle = math.pi * (1 - t)
		local localOffset = Vector3.new(math.cos(angle) * ARC_R, 0, math.sin(angle) * ARC_R)
		local worldPoint = frame:PointToWorldSpace(localOffset)
		if isClear(worldPoint) then
			table.insert(points, worldPoint)
			mark(worldPoint, Color3.fromRGB(255, 200, 0))
		end
	end

	return points
end

------------------------------------------------------------
--  Path Builder
------------------------------------------------------------
local y0 = prt.Position.Y
local xMin = GRID_X_OFFSET
local xMax = GRID_X_OFFSET + GRID_WIDTH
local path = {}
local lines = {}
local rowDirections = {}

for i = 0, NUM_ROWS - 1 do
	local zStart = GRID_Z_OFFSET + Z_OFFSET + i * (LINE_HEIGHT + ARC_TOTAL)
	local rowDir = (i % 2 == 0) and 1 or -1
	table.insert(rowDirections, rowDir)

	local line = {}
	if rowDir == 1 then
		for x = xMin, xMax, GRID do
			local p = Vector3.new(x, y0, zStart)
			if isClear(p) then table.insert(line, p); mark(p, Color3.fromRGB(0, 255, 0)) end
		end
	else
		for x = xMax, xMin, -GRID do
			local p = Vector3.new(x, y0, zStart)
			if isClear(p) then table.insert(line, p); mark(p, Color3.fromRGB(0, 255, 0)) end
		end
	end

	table.insert(lines, line)
end

for i = 1, #lines do
	local line = lines[i]
	for _, p in ipairs(line) do
		table.insert(path, p)
	end

	local nextLine = lines[i + 1]
	if nextLine and #line > 0 and #nextLine > 0 then
		local rowEnd = line[#line]
		local rowStart = nextLine[1]
		local flipArc = (i % 2 == 1)
		local arcPts = buildArcBetween(rowEnd, rowStart, 28, flipArc)

		local currentRowDir = rowDirections[i]
		if currentRowDir == 1 then
			for j = #arcPts, 1, -1 do
				table.insert(path, arcPts[j])
			end
		else
			for _, p in ipairs(arcPts) do
				table.insert(path, p)
			end
		end
	end
end

print("Path pts:", #path)

local idx = 1
RunService.Heartbeat:Connect(function(dt)
	if idx > #path then return end
	local pos = prt.Position
	local tgt = path[idx]
	local flat = Vector3.new(tgt.X, pos.Y, tgt.Z)
	local dir = flat - pos

	if dir.Magnitude < 1 then
		idx += 1
		return
	end
	------------------------------------------------------------
	--This is the part of the code that implements the Euler method
	------------------------------------------------------------
	local desire = dir.Unit * SPEED
	vForce.Force = (desire - prt.Velocity) * MASS / dt

	local yawDir = Vector3.new(dir.X, 0, dir.Z).Unit
	if yawDir.Magnitude > 0 then
		gyro.CFrame = gyro.CFrame:Lerp(CFrame.new(pos, pos + yawDir), 0.2)
	end
end)
