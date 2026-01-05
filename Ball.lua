--!strict

--[[
	BALL PHYSICS SYSTEM WITH CUSTOM SIMULATION
	
	This module implements a deterministic physics engine for balls using fixed timestep simulation.
	Instead of relying on Roblox's built-in physics, it runs custom calculations every frame to
	achieve precise control over ball behavior, collision response, and rolling mechanics.
	
	The system works by:
	1. Breaking each frame into fixed substeps (1/120 seconds) for consistent physics
	2. Using Spherecast to predict collisions before they happen
	3. Calculating velocity changes based on collision normals and material properties
	4. Converting linear velocity to angular velocity for realistic rolling
	5. Syncing visual rotation with physical motion via CFrame manipulation
]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

local Assets = ReplicatedStorage:FindFirstChild("Assets")
local BallAsset = Assets:FindFirstChild("Ball") :: BasePart -- Just an sphere

local RemoteEvents = ReplicatedStorage:FindFirstChild("RemoteEvents")
local BallEvents = RemoteEvents:FindFirstChild("Ball")
local UpdateEvent = BallEvents:FindFirstChild("Update") :: UnreliableRemoteEvent

-- Splits each frame into smaller fixed timesteps to ensure physics calculations remain
-- deterministic regardless of framerate variations
local SUBSTEP_TIME: number = 1 / 120

-- Controls how much kinetic energy remains after a bounce (0.7 = 70% energy retained)
local BOUNCINESS: number = 0.7

-- Determines how much lateral velocity is lost when sliding along a surface during collision
local FRICTION: number = 0.85

-- Gradually reduces angular velocity over time, simulating surface resistance during rolling
local ROLLING_FRICTION: number = 0.75

-- Used in moment of inertia calculation (2/5 is correct for a uniform solid sphere)
local INERTIA_FACTOR: number = 2 / 5

-- Velocity threshold below which the ball transitions from bouncing to pure rolling motion
local MIN_ROLL_SPEED: number = 1.0

-- Angular velocity threshold below which rotation is considered negligible and set to zero
local MIN_SPIN_SPEED: number = 0.1

-- Dot product threshold for detecting flat ground (0.9 means normal is nearly vertical)
local FLAT_GROUND_ANGLE: number = 0.9

-- Extra distance added to raycasts to catch collisions slightly before actual contact
local COLLISION_BUFFER: number = 0.1

-- Additional downward raycast distance to detect ground when ball is nearly stationary
local GROUND_CHECK_OFFSET: number = 0.05

-- Maximum downward velocity that can be converted to rolling (prevents high falls from becoming rolls)
local MAX_GROUND_IMPACT: number = -10

local ROUND_DECIMALS: number = 3

local activeBalls: { ClassType } = {}
local nextBallId: number = 1

--- Reduces floating point precision to avoid microscopic differences accumulating over time
local function roundVector3(vector: Vector3, decimals: number): Vector3
	local multiplier: number = 10 ^ decimals
	return Vector3.new(
		math.floor(vector.X * multiplier + 0.5) / multiplier,
		math.floor(vector.Y * multiplier + 0.5) / multiplier,
		math.floor(vector.Z * multiplier + 0.5) / multiplier
	)
end

--- Searches the active balls table linearly to find a ball by its unique ID
local function getBallById(ballId: number): ClassType?
	for _, ball in activeBalls do
		if ball.id == ballId then
			return ball
		end
	end
	return nil
end

local Ball = {}
Ball.__index = Ball

export type ClassInterface = {
	id: number,
	instance: BasePart,
	shouldSimulate: boolean,
	heartbeatConnection: RBXScriptConnection?,
	velocity: Vector3,
	angularVelocity: Vector3,
	radius: number,
	bounciness: number,
	friction: number,
	gravity: Vector3,
	momentOfInertia: number,
	rollingFriction: number,
}
export type ClassType = typeof(setmetatable({} :: ClassInterface, Ball))

--- Creates a ball instance and initializes all physics properties to their defaults
--- The ball won't simulate until Start() is called explicitly
function Ball.new(position: Vector3): ClassType
	local self = setmetatable({}, Ball) :: any

	self.id = nextBallId
	nextBallId += 1

	self.instance = BallAsset:Clone()
	self.instance.Position = position
	self.instance:SetAttribute("id", self.id)
	self.instance.Parent = workspace.Ball -- Folder

	self.shouldSimulate = true
	self.heartbeatConnection = nil

	self.velocity = Vector3.zero
	self.angularVelocity = Vector3.zero
	self.radius = self.instance.Size.X / 2
	self.bounciness = BOUNCINESS
	self.friction = FRICTION
	self.gravity = Vector3.new(0, -workspace.Gravity, 0)
	self.momentOfInertia = INERTIA_FACTOR * (self.radius ^ 2)
	self.rollingFriction = ROLLING_FRICTION

	table.insert(activeBalls, self :: ClassType)

	UpdateEvent:FireAllClients(self.id, "N", self.instance.CFrame)

	return self
end

--- Runs one physics substep by predicting the ball's path, detecting collisions, and updating
--- position/velocity accordingly. Uses Spherecast to check ahead of movement rather than
--- detecting collisions after they occur
function Ball._simulate(self: ClassType, deltaTime: number)
	if not self.shouldSimulate then
		return
	end

	local currentPosition: Vector3 = self.instance.Position
	local velocityStep: Vector3 = self.velocity * deltaTime

	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = { self.instance, workspace.Characters }
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude

	-- Determines where to cast based on current movement. If moving, cast in movement direction
	-- If stationary, cast downward to detect ground below
	local velocityMagnitude: number = velocityStep.Magnitude
	local castDirection: Vector3

	if velocityMagnitude > 0 then
		castDirection = velocityStep.Unit * (velocityMagnitude + COLLISION_BUFFER)
	else
		castDirection = Vector3.new(0, -COLLISION_BUFFER, 0)
	end

	-- Spherecast projects a sphere along a ray, detecting any surfaces the ball would hit
	-- during this substep
	local spherecastResult = workspace:Spherecast(
		currentPosition,
		self.radius,
		castDirection,
		raycastParams
	)

	if spherecastResult then
		self:_handleCollision(spherecastResult.Position, spherecastResult.Normal)
	else
		-- No collision detected, so apply gravity and update position freely
		self.velocity += self.gravity * deltaTime
		local newPosition: Vector3 = self.instance.Position + velocityStep

		-- Additional horizontal cast catches walls that might be missed by the main spherecast
		-- when moving mostly vertically
		local horizontalVelocity: Vector3 = Vector3.new(velocityStep.X, 0, velocityStep.Z)
		local horizontalSpeed: number = horizontalVelocity.Magnitude

		if horizontalSpeed > 0 then
			local horizontalDirection: Vector3 = horizontalVelocity.Unit
			local horizontalDistance: number = horizontalSpeed + self.radius + GROUND_CHECK_OFFSET

			local horizontalCastResult = workspace:Spherecast(
				self.instance.Position,
				self.radius,
				horizontalDirection * horizontalDistance,
				raycastParams
			)

			if horizontalCastResult then
				self:_handleCollision(horizontalCastResult.Position, horizontalCastResult.Normal)
			else
				self.instance.Position = newPosition
			end
		else
			self.instance.Position = newPosition
		end

		-- Separate downward raycast ensures the ball doesn't clip through the ground when
		-- falling slowly or stationary
		local groundRaycast = workspace:Raycast(
			self.instance.Position,
			Vector3.new(0, -self.radius - GROUND_CHECK_OFFSET, 0),
			raycastParams
		)

		if groundRaycast then
			local groundY: number = groundRaycast.Position.Y
			local ballBottomY: number = self.instance.Position.Y - self.radius

			if ballBottomY < groundY then
				self.instance.Position = Vector3.new(
					self.instance.Position.X,
					groundY + self.radius,
					self.instance.Position.Z
				)

				-- Converts downward velocity into horizontal rolling motion when landing softly
				if self.velocity.Y < 0 and self.velocity.Y > MAX_GROUND_IMPACT then
					self.velocity = Vector3.new(self.velocity.X, 0, self.velocity.Z)

					local linearSpeed: number = self.velocity.Magnitude
					if linearSpeed > 0 then
						-- Angular velocity is perpendicular to movement direction (right-hand rule)
						local rollAxis: Vector3 = Vector3.new(-self.velocity.Z, 0, self.velocity.X).Unit
						self.angularVelocity = rollAxis * (linearSpeed / self.radius)
					else
						self.angularVelocity = Vector3.zero
					end
				end
			end
		end
	end

	self:_applyRollingFriction(deltaTime)

	-- Synchronizes linear velocity with angular velocity to maintain consistent rolling
	if self.angularVelocity.Magnitude > 0 then
		local linearVelocityFromRolling: Vector3 = self.angularVelocity:Cross(
			Vector3.new(0, -self.radius, 0)
		)
		self.velocity = Vector3.new(
			linearVelocityFromRolling.X,
			self.velocity.Y,
			linearVelocityFromRolling.Z
		)
	end

	self:_updateRotation(deltaTime)
end

--- Calculates the ball's response to hitting a surface by reflecting velocity along the
--- collision normal, applying energy loss from bounciness, and reducing tangential velocity
--- through friction
function Ball._handleCollision(self: ClassType, hitPosition: Vector3, normal: Vector3)
	-- Moves ball just outside the surface to prevent embedding
	self.instance.Position = hitPosition + normal * self.radius

	-- Reflects velocity vector across the surface normal (standard reflection formula)
	local reflectedVelocity: Vector3 = self.velocity - 2 * self.velocity:Dot(normal) * normal
	local bouncedVelocity: Vector3 = reflectedVelocity * self.bounciness

	-- Separates velocity into normal and tangential components, then reduces tangential
	-- component to simulate surface friction
	local tangentialComponent: Vector3 = bouncedVelocity - bouncedVelocity:Dot(normal) * normal
	bouncedVelocity -= tangentialComponent * (1 - self.friction)

	-- Transitions to rolling behavior when bouncing becomes negligible and surface is flat
	if bouncedVelocity.Magnitude < MIN_ROLL_SPEED and normal.Y > FLAT_GROUND_ANGLE then
		self:_handleGroundContact(normal)
	else
		self.velocity = bouncedVelocity

		-- Generates torque when rolling on slopes, causing the ball to spin faster downhill
		local slopeTorque: Vector3 = normal:Cross(self.gravity) * self.radius / self.momentOfInertia
		self.angularVelocity += slopeTorque * (1 / 120)
	end
end

--- Transitions the ball from bouncing physics to rolling physics by zeroing vertical velocity
--- and calculating appropriate angular velocity based on horizontal motion
function Ball._handleGroundContact(self: ClassType, normal: Vector3)
	self.velocity = Vector3.new(self.velocity.X, 0, self.velocity.Z)

	local linearSpeed: number = self.velocity.Magnitude
	if linearSpeed > 0 then
		-- Perpendicular axis ensures the ball rotates in the direction of movement
		local rollAxis: Vector3 = Vector3.new(-self.velocity.Z, 0, self.velocity.X).Unit
		self.angularVelocity = rollAxis * (linearSpeed / self.radius)
	else
		self.angularVelocity = Vector3.zero
	end
end

--- Gradually reduces angular velocity over time using exponential decay, simulating
--- resistance from the rolling surface
function Ball._applyRollingFriction(self: ClassType, deltaTime: number)
	if self.angularVelocity.Magnitude > 0 then
		-- Multiplies angular velocity by a value slightly less than 1, reducing it each frame
		local frictionScale: number = math.max(0, 1 - self.rollingFriction * deltaTime)
		self.angularVelocity *= frictionScale

		-- Snaps to zero when rotation becomes imperceptible to avoid endless micro-movements
		if self.angularVelocity.Magnitude < MIN_SPIN_SPEED then
			self.angularVelocity = Vector3.zero
			self.velocity = Vector3.new(0, self.velocity.Y, 0)
		end
	end
end

--- Rotates the ball's visual mesh to match its physical angular velocity using
--- axis-angle rotation
function Ball._updateRotation(self: ClassType, deltaTime: number)
	if self.angularVelocity.Magnitude > 0 then
		-- Converts angular velocity into rotation angle for this frame
		local rotationAngle: number = self.angularVelocity.Magnitude * deltaTime
		local rotationAxis: Vector3 = self.angularVelocity.Unit
		local currentCFrame: CFrame = self.instance.CFrame

		-- Negative angle accounts for the relationship between world space movement
		-- and object space rotation
		local rotation: CFrame = CFrame.fromAxisAngle(rotationAxis, -rotationAngle)
		self.instance.CFrame = currentCFrame * rotation
	end
end

--- Sends the ball's current CFrame to all clients via UnreliableRemoteEvent for replication
function Ball._updateClients(self: ClassType)
	UpdateEvent:FireAllClients(self.id, self.instance.CFrame)
end

--- Processes one frame of simulation by subdividing the frame delta into multiple fixed
--- substeps, ensuring physics remains stable regardless of framerate
function Ball._tick(self: ClassType, deltaTime: number)
	local remainingTime: number = deltaTime

	-- Breaks variable frame time into consistent substeps for deterministic simulation
	while remainingTime > 0 do
		local stepTime: number = math.min(SUBSTEP_TIME, remainingTime)
		self:_simulate(stepTime)
		remainingTime -= stepTime
	end

	-- Automatically cleans up balls that fall below the world's kill plane
	if self.instance.Position.Y < workspace.FallenPartsDestroyHeight then
		self:Delete()
		return
	end

	self:_updateClients()
end

--- Connects the ball to RunService.Heartbeat to begin running physics every frame
function Ball.Start(self: ClassType)
	self.heartbeatConnection = RunService.Heartbeat:Connect(function(deltaTime: number)
		self:_tick(deltaTime)
	end)
end

--- Removes the ball from the simulation, destroys its instance, and disconnects all events
function Ball.Delete(self: ClassType)
	for index, ball in activeBalls do
		if ball == self then
			table.remove(activeBalls, index)
			break
		end
	end

	self.instance:Destroy()

	if self.heartbeatConnection then
		self.heartbeatConnection:Disconnect()
		self.heartbeatConnection = nil
	end
end

--- Adds external velocity to the ball and recalculates angular velocity to maintain
--- consistent rolling behavior with the new linear motion
function Ball.ApplyVelocity(self: ClassType, velocity: Vector3)
	self.velocity += velocity

	-- Extracts horizontal component since only horizontal movement generates rolling
	local horizontalVelocity: Vector3 = Vector3.new(self.velocity.X, 0, self.velocity.Z)
	local linearSpeed: number = horizontalVelocity.Magnitude

	if linearSpeed > 0 then
		local rollAxis: Vector3 = Vector3.new(-horizontalVelocity.Z, 0, horizontalVelocity.X).Unit
		self.angularVelocity = rollAxis * (linearSpeed / self.radius)
	else
		self.angularVelocity = Vector3.zero
	end
end

--- Returns the current world position of the ball's instance
function Ball.GetPosition(self: ClassType): Vector3
	return self.instance.Position
end

return {
	new = Ball.new,
	getBallById = getBallById,
}
