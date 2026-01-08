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
	
	EXECUTION FLOW:
	- Ball.new() creates a new ball instance with default physics properties
	- Ball.Start() connects the ball to RunService.Heartbeat for per-frame updates
	- Each frame, _tick() divides the deltaTime into fixed substeps
	- Each substep calls _simulate() which:
	  a) Casts a sphere along the velocity vector to detect upcoming collisions
	  b) If collision found, calls _handleCollision() to reflect velocity and apply friction
	  c) If no collision, applies gravity and updates position
	  d) Applies rolling friction to gradually slow rotation
	  e) Synchronizes linear and angular velocity for consistent rolling
	  f) Updates the visual rotation via _updateRotation()
	- After all substeps, _updateClients() replicates the new position to clients
]]

local ReplicatedStorage = game:GetService("ReplicatedStorage")
local RunService = game:GetService("RunService")

-- Reference to the ball asset that will be cloned for each new ball instance
local Assets = ReplicatedStorage:FindFirstChild("Assets")
local BallAsset = Assets:FindFirstChild("Ball") :: BasePart -- Just an sphere

-- Remote events used to replicate ball updates from server to clients
local RemoteEvents = ReplicatedStorage:FindFirstChild("RemoteEvents")
local BallEvents = RemoteEvents:FindFirstChild("Ball")
local UpdateEvent = BallEvents:FindFirstChild("Update") :: UnreliableRemoteEvent

-- PHYSICS CONSTANTS --

-- Splits each frame into smaller fixed timesteps to ensure physics calculations remain
-- deterministic regardless of framerate variations. A smaller timestep means more precise
-- collision detection but higher computational cost. 1/120 seconds balances accuracy and performance.
local SUBSTEP_TIME: number = 1 / 120

-- Controls how much kinetic energy remains after a bounce (0.7 = 70% energy retained, 30% lost).
-- Values closer to 1.0 create bouncier balls, values closer to 0.0 create dead bounces.
local BOUNCINESS: number = 0.7

-- Determines how much lateral velocity is lost when sliding along a surface during collision.
-- Higher values (closer to 1.0) mean less friction and more sliding. Lower values mean more grip.
local FRICTION: number = 0.85

-- Gradually reduces angular velocity over time, simulating surface resistance during rolling.
-- This value is multiplied by deltaTime and subtracted from the friction scale each frame.
local ROLLING_FRICTION: number = 0.75

-- Used in moment of inertia calculation (2/5 is the correct physical constant for a uniform solid sphere).
-- This affects how easily the ball's rotation can be changed by external forces.
local INERTIA_FACTOR: number = 2 / 5

-- Velocity threshold below which the ball transitions from bouncing to pure rolling motion.
-- When bounce velocity drops below this value on flat ground, the ball stops bouncing and rolls.
local MIN_ROLL_SPEED: number = 1.0

-- Angular velocity threshold below which rotation is considered negligible and set to zero.
-- This prevents infinite micro-rotations that would never fully stop the ball.
local MIN_SPIN_SPEED: number = 0.1

-- Dot product threshold for detecting flat ground (0.9 means normal is nearly vertical).
-- The dot product of a surface normal with Vector3.new(0,1,0) gives the "flatness" of the surface.
local FLAT_GROUND_ANGLE: number = 0.9

-- Extra distance added to raycasts to catch collisions slightly before actual contact.
-- This prevents the ball from clipping into surfaces at high speeds.
local COLLISION_BUFFER: number = 0.1

-- Additional downward raycast distance to detect ground when ball is nearly stationary.
-- Without this, a slow-moving ball might not detect the ground directly below it.
local GROUND_CHECK_OFFSET: number = 0.05

-- Maximum downward velocity that can be converted to rolling (prevents high falls from becoming rolls).
-- Falls faster than this value will bounce instead of transitioning directly to rolling.
local MAX_GROUND_IMPACT: number = -10

-- Number of decimal places to round vector components to for network replication consistency.
local ROUND_DECIMALS: number = 3

-- Table storing all currently active ball instances in the simulation
local activeBalls: { ClassType } = {}

-- Counter that increments with each new ball to provide unique IDs
local nextBallId: number = 1

--- Reduces floating point precision to avoid microscopic differences accumulating over time.
--- This is important for network replication to ensure server and client states don't drift.
--- 
--- @param vector Vector3 - The vector to round
--- @param decimals number - Number of decimal places to preserve
--- @return Vector3 - A new vector with rounded components
local function roundVector3(vector: Vector3, decimals: number): Vector3
	-- Calculate the multiplier once (e.g., 1000 for 3 decimals)
	local multiplier: number = 10 ^ decimals
	
	-- For each component: multiply, add 0.5 for proper rounding, floor, then divide back
	-- This achieves standard rounding behavior (0.5 rounds up)
	return Vector3.new(
		math.floor(vector.X * multiplier + 0.5) / multiplier,
		math.floor(vector.Y * multiplier + 0.5) / multiplier,
		math.floor(vector.Z * multiplier + 0.5) / multiplier
	)
end

--- Searches the active balls table linearly to find a ball by its unique ID.
--- Used by other systems (like client code) to retrieve specific ball instances.
--- 
--- @param ballId number - The unique ID to search for
--- @return ClassType? - The ball instance if found, nil otherwise
local function getBallById(ballId: number): ClassType?
	-- Iterate through all active balls
	for _, ball in activeBalls do
		-- Check if this ball's ID matches the search ID
		if ball.id == ballId then
			return ball
		end
	end
	-- No ball found with this ID
	return nil
end

-- BALL CLASS DEFINITION --

local Ball = {}
Ball.__index = Ball

-- Type definitions for the Ball class
export type ClassInterface = {
	id: number,                           -- Unique identifier for this ball
	instance: BasePart,                   -- The physical part in the workspace
	shouldSimulate: boolean,              -- Whether physics should run for this ball
	heartbeatConnection: RBXScriptConnection?, -- Connection to RunService.Heartbeat
	velocity: Vector3,                    -- Current linear velocity in studs/second
	angularVelocity: Vector3,            -- Current angular velocity in radians/second
	radius: number,                       -- Ball radius in studs
	bounciness: number,                   -- Energy retention coefficient (0-1)
	friction: number,                     -- Surface friction coefficient (0-1)
	gravity: Vector3,                     -- Gravitational acceleration vector
	momentOfInertia: number,             -- Rotational inertia (affects spin behavior)
	rollingFriction: number,             -- Rolling resistance coefficient
}
export type ClassType = typeof(setmetatable({} :: ClassInterface, Ball))

--- Creates a ball instance and initializes all physics properties to their defaults.
--- The ball won't simulate until Start() is called explicitly.
--- 
--- EXECUTION FLOW:
--- 1. Creates a new table and sets Ball as its metatable
--- 2. Assigns a unique ID and increments the counter
--- 3. Clones the ball asset and positions it in the workspace
--- 4. Initializes all physics properties (velocity, radius, friction, etc.)
--- 5. Adds the ball to the activeBalls tracking table
--- 6. Replicates creation to all clients via RemoteEvent
--- 
--- @param position Vector3 - The initial world position for the ball
--- @return ClassType - The newly created ball instance
function Ball.new(position: Vector3): ClassType
	-- Create the instance table with Ball metatable for inheritance
	local self = setmetatable({}, Ball) :: any

	-- Assign unique ID and increment for next ball
	self.id = nextBallId
	nextBallId += 1

	-- Clone the ball asset from ReplicatedStorage and set its initial position
	self.instance = BallAsset:Clone()
	self.instance.Position = position
	
	-- Store the ID as an attribute for easy lookup by clients
	self.instance:SetAttribute("id", self.id)
	
	-- Parent to workspace to make it visible (assumes workspace.Ball folder exists)
	self.instance.Parent = workspace.Ball

	-- Physics will start when Start() is called
	self.shouldSimulate = true
	self.heartbeatConnection = nil

	-- Initialize velocity vectors to zero (ball starts stationary)
	self.velocity = Vector3.zero
	self.angularVelocity = Vector3.zero
	
	-- Calculate radius from the ball's size (assumes uniform sphere)
	self.radius = self.instance.Size.X / 2
	
	-- Set physics material properties from constants
	self.bounciness = BOUNCINESS
	self.friction = FRICTION
	
	-- Create gravity vector using workspace gravity setting
	self.gravity = Vector3.new(0, -workspace.Gravity, 0)
	
	-- Calculate moment of inertia: I = (2/5) * m * r^2 (mass cancels out in our calculations)
	self.momentOfInertia = INERTIA_FACTOR * (self.radius ^ 2)
	
	self.rollingFriction = ROLLING_FRICTION

	-- Add to tracking table so the ball can be found by ID
	table.insert(activeBalls, self :: ClassType)

	-- Replicate the ball creation to all clients
	-- "N" indicates this is a New ball event
	UpdateEvent:FireAllClients(self.id, "N", self.instance.CFrame)

	return self
end

--- Runs one physics substep by predicting the ball's path, detecting collisions, and updating
--- position/velocity accordingly. Uses Spherecast to check ahead of movement rather than
--- detecting collisions after they occur.
--- 
--- EXECUTION FLOW:
--- 1. Check if simulation is enabled, return early if not
--- 2. Calculate where the ball wants to move this substep (velocityStep)
--- 3. Set up raycast parameters to ignore the ball itself and characters
--- 4. Determine cast direction (movement direction if moving, downward if stationary)
--- 5. Perform spherecast to detect collisions along the path
--- 6. If collision detected:
---    - Call _handleCollision() to process the bounce/roll response
--- 7. If no collision:
---    a) Apply gravity to velocity
---    b) Calculate new position
---    c) Perform additional horizontal spherecast to catch walls
---    d) Perform additional downward raycast to catch ground
---    e) If ground detected and falling softly, transition to rolling
--- 8. Apply rolling friction to gradually slow the ball
--- 9. Synchronize linear velocity with angular velocity
--- 10. Update visual rotation
--- 
--- @param deltaTime number - The time elapsed for this substep (typically 1/120 seconds)
function Ball._simulate(self: ClassType, deltaTime: number)
	-- Early exit if this ball shouldn't be simulating
	if not self.shouldSimulate then
		return
	end

	-- Store current position for reference
	local currentPosition: Vector3 = self.instance.Position
	
	-- Calculate how far the ball wants to move this substep based on current velocity
	local velocityStep: Vector3 = self.velocity * deltaTime

	-- Create raycast parameters to filter out objects we don't want to collide with
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = { self.instance, workspace.Characters }
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude

	-- Determine cast direction based on current movement state
	-- If moving: cast in movement direction plus a buffer
	-- If stationary: cast downward to detect ground below
	local velocityMagnitude: number = velocityStep.Magnitude
	local castDirection: Vector3

	if velocityMagnitude > 0 then
		-- Ball is moving, cast in the direction of movement
		-- Add COLLISION_BUFFER to catch collisions slightly before they occur
		castDirection = velocityStep.Unit * (velocityMagnitude + COLLISION_BUFFER)
	else
		-- Ball is stationary, cast straight down to detect ground
		castDirection = Vector3.new(0, -COLLISION_BUFFER, 0)
	end

	-- Spherecast projects a sphere along a ray, detecting any surfaces the ball would hit
	-- during this substep. This is more accurate than a simple raycast for spherical objects.
	local spherecastResult = workspace:Spherecast(
		currentPosition,
		self.radius,
		castDirection,
		raycastParams
	)

	-- Process the spherecast result
	if spherecastResult then
		-- Collision detected! Handle the bounce/roll response
		self:_handleCollision(spherecastResult.Position, spherecastResult.Normal)
	else
		-- No collision in main cast, proceed with free movement
		
		-- Apply gravitational acceleration to velocity
		self.velocity += self.gravity * deltaTime
		
		-- Calculate the new position by adding velocity step to current position
		local newPosition: Vector3 = self.instance.Position + velocityStep

		-- Additional horizontal cast catches walls that might be missed by the main spherecast
		-- when moving mostly vertically. This prevents clipping through walls when falling.
		
		-- Extract only horizontal components of velocity (ignore Y)
		local horizontalVelocity: Vector3 = Vector3.new(velocityStep.X, 0, velocityStep.Z)
		local horizontalSpeed: number = horizontalVelocity.Magnitude

		if horizontalSpeed > 0 then
			-- There is horizontal movement, perform a horizontal spherecast
			local horizontalDirection: Vector3 = horizontalVelocity.Unit
			
			-- Cast slightly further than the movement to catch nearby walls
			local horizontalDistance: number = horizontalSpeed + self.radius + GROUND_CHECK_OFFSET

			local horizontalCastResult = workspace:Spherecast(
				self.instance.Position,
				self.radius,
				horizontalDirection * horizontalDistance,
				raycastParams
			)

			if horizontalCastResult then
				-- Hit a wall, process collision
				self:_handleCollision(horizontalCastResult.Position, horizontalCastResult.Normal)
			else
				-- No wall hit, safe to move to new position
				self.instance.Position = newPosition
			end
		else
			-- No horizontal movement, just update position
			self.instance.Position = newPosition
		end

		-- Separate downward raycast ensures the ball doesn't clip through the ground when
		-- falling slowly or stationary. Spherecast might miss the ground if velocity is very small.
		local groundRaycast = workspace:Raycast(
			self.instance.Position,
			Vector3.new(0, -self.radius - GROUND_CHECK_OFFSET, 0),
			raycastParams
		)

		if groundRaycast then
			-- Ground detected below the ball
			local groundY: number = groundRaycast.Position.Y
			
			-- Calculate where the bottom of the ball currently is
			local ballBottomY: number = self.instance.Position.Y - self.radius

			-- Check if the ball has clipped below the ground surface
			if ballBottomY < groundY then
				-- Correct the position to rest exactly on the ground surface
				self.instance.Position = Vector3.new(
					self.instance.Position.X,
					groundY + self.radius,  -- Position ball so its bottom touches ground
					self.instance.Position.Z
				)

				-- Converts downward velocity into horizontal rolling motion when landing softly
				-- Only applies if falling speed is within the rolling threshold
				if self.velocity.Y < 0 and self.velocity.Y > MAX_GROUND_IMPACT then
					-- Zero out vertical velocity (no longer falling)
					self.velocity = Vector3.new(self.velocity.X, 0, self.velocity.Z)

					-- Calculate angular velocity for rolling based on horizontal speed
					local linearSpeed: number = self.velocity.Magnitude
					if linearSpeed > 0 then
						-- Angular velocity is perpendicular to movement direction (right-hand rule)
						-- Cross product of Y-axis with velocity gives rotation axis
						local rollAxis: Vector3 = Vector3.new(-self.velocity.Z, 0, self.velocity.X).Unit
						
						-- v = ω × r, so ω = v / r
						self.angularVelocity = rollAxis * (linearSpeed / self.radius)
					else
						-- Not moving horizontally, no rotation
						self.angularVelocity = Vector3.zero
					end
				end
			end
		end
	end

	-- Apply rolling friction to gradually slow down rotation
	self:_applyRollingFriction(deltaTime)

	-- Synchronize linear velocity with angular velocity to maintain consistent rolling
	-- This ensures the ball's movement matches its rotation (no sliding)
	if self.angularVelocity.Magnitude > 0 then
		-- Calculate linear velocity from angular velocity: v = ω × r
		local linearVelocityFromRolling: Vector3 = self.angularVelocity:Cross(
			Vector3.new(0, -self.radius, 0)  -- Radius vector pointing down
		)
		
		-- Update horizontal velocity to match rolling, preserve vertical velocity (gravity)
		self.velocity = Vector3.new(
			linearVelocityFromRolling.X,
			self.velocity.Y,
			linearVelocityFromRolling.Z
		)
	end

	-- Update the visual rotation of the ball to match its angular velocity
	self:_updateRotation(deltaTime)
end

--- Calculates the ball's response to hitting a surface by reflecting velocity along the
--- collision normal, applying energy loss from bounciness, and reducing tangential velocity
--- through friction.
--- 
--- EXECUTION FLOW:
--- 1. Position ball exactly at collision point (prevents embedding in surface)
--- 2. Calculate reflected velocity using reflection formula: v' = v - 2(v·n)n
--- 3. Apply bounciness to reduce energy in the bounce
--- 4. Separate velocity into normal and tangential components
--- 5. Apply friction to reduce tangential (sliding) velocity
--- 6. Check if bounce is small enough and surface is flat enough to transition to rolling
--- 7. If transitioning to rolling: call _handleGroundContact()
--- 8. If still bouncing: apply slope torque to spin the ball on angled surfaces
--- 
--- @param hitPosition Vector3 - The point where the collision occurred
--- @param normal Vector3 - The surface normal at the collision point (points away from surface)
function Ball._handleCollision(self: ClassType, hitPosition: Vector3, normal: Vector3)
	-- Moves ball just outside the surface to prevent embedding
	-- Position = collision point + (normal direction × ball radius)
	self.instance.Position = hitPosition + normal * self.radius

	-- Reflects velocity vector across the surface normal using standard reflection formula
	-- Formula: v_reflected = v - 2(v·n)n
	-- Where v·n is the dot product (component of velocity along normal)
	local reflectedVelocity: Vector3 = self.velocity - 2 * self.velocity:Dot(normal) * normal
	
	-- Apply bounciness factor to simulate energy loss
	-- bounciness = 0.7 means 70% of energy retained, 30% lost
	local bouncedVelocity: Vector3 = reflectedVelocity * self.bounciness

	-- Separate velocity into normal and tangential components, then reduce tangential
	-- component to simulate surface friction (sliding resistance)
	
	-- Normal component: projection of velocity onto the normal
	local normalComponent: Vector3 = bouncedVelocity:Dot(normal) * normal
	
	-- Tangential component: what's left after removing normal component
	local tangentialComponent: Vector3 = bouncedVelocity - normalComponent
	
	-- Reduce tangential velocity by friction factor
	-- (1 - friction) = amount of tangential velocity lost
	bouncedVelocity -= tangentialComponent * (1 - self.friction)

	-- Transitions to rolling behavior when bouncing becomes negligible and surface is flat
	-- MIN_ROLL_SPEED check: is bounce weak enough?
	-- normal.Y > FLAT_GROUND_ANGLE check: is surface flat enough? (dot product with up vector)
	if bouncedVelocity.Magnitude < MIN_ROLL_SPEED and normal.Y > FLAT_GROUND_ANGLE then
		-- Bounce is done, transition to pure rolling
		self:_handleGroundContact(normal)
	else
		-- Still bouncing, update velocity
		self.velocity = bouncedVelocity

		-- Generates torque when rolling on slopes, causing the ball to spin faster downhill
		-- Cross product of normal and gravity gives the direction of slope-induced rotation
		-- Divided by moment of inertia to convert torque to angular acceleration
		local slopeTorque: Vector3 = normal:Cross(self.gravity) * self.radius / self.momentOfInertia
		
		-- Apply torque over the substep time (1/120 second)
		self.angularVelocity += slopeTorque * (1 / 120)
	end
end

--- Transitions the ball from bouncing physics to rolling physics by zeroing vertical velocity
--- and calculating appropriate angular velocity based on horizontal motion.
--- 
--- EXECUTION FLOW:
--- 1. Remove vertical component of velocity (stop bouncing)
--- 2. Calculate horizontal speed
--- 3. If moving horizontally:
---    a) Calculate roll axis perpendicular to movement direction
---    b) Calculate angular velocity using v = ω × r formula
--- 4. If not moving: zero out angular velocity
--- 
--- @param normal Vector3 - The surface normal (currently unused but kept for future features)
function Ball._handleGroundContact(self: ClassType, normal: Vector3)
	-- Zero out vertical velocity component, keep horizontal components
	-- This stops the ball from bouncing and makes it roll instead
	self.velocity = Vector3.new(self.velocity.X, 0, self.velocity.Z)

	-- Calculate the horizontal speed (magnitude of horizontal velocity)
	local linearSpeed: number = self.velocity.Magnitude
	
	if linearSpeed > 0 then
		-- Ball is moving horizontally, calculate appropriate angular velocity
		
		-- Calculate roll axis perpendicular to movement direction using cross product
		-- For a ball rolling in direction (x, 0, z), rotation axis is (-z, 0, x)
		-- This follows the right-hand rule for rotation
		local rollAxis: Vector3 = Vector3.new(-self.velocity.Z, 0, self.velocity.X).Unit
		
		-- Calculate angular velocity magnitude from linear velocity
		-- Formula: ω = v / r (derived from v = ω × r for rolling without slipping)
		self.angularVelocity = rollAxis * (linearSpeed / self.radius)
	else
		-- Ball is not moving, no rotation needed
		self.angularVelocity = Vector3.zero
	end
end

--- Gradually reduces angular velocity over time using exponential decay, simulating
--- resistance from the rolling surface.
--- 
--- EXECUTION FLOW:
--- 1. Check if ball is rotating
--- 2. Calculate friction scale factor based on rolling friction coefficient and time
--- 3. Multiply angular velocity by scale factor (exponential decay)
--- 4. If angular velocity drops below threshold:
---    a) Snap angular velocity to zero
---    b) Zero out horizontal velocity (ball stops moving)
--- 
--- @param deltaTime number - Time elapsed this substep
function Ball._applyRollingFriction(self: ClassType, deltaTime: number)
	if self.angularVelocity.Magnitude > 0 then
		-- Multiplies angular velocity by a value slightly less than 1, reducing it each frame
		-- Formula: scale = max(0, 1 - friction * deltaTime)
		-- This creates exponential decay: each frame reduces velocity by a percentage
		local frictionScale: number = math.max(0, 1 - self.rollingFriction * deltaTime)
		self.angularVelocity *= frictionScale

		-- Snaps to zero when rotation becomes imperceptible to avoid endless micro-movements
		-- Without this, the ball would rotate at infinitesimally small speeds forever
		if self.angularVelocity.Magnitude < MIN_SPIN_SPEED then
			-- Rotation is negligible, stop completely
			self.angularVelocity = Vector3.zero
			
			-- Also stop horizontal movement (preserve vertical for gravity)
			self.velocity = Vector3.new(0, self.velocity.Y, 0)
		end
	end
end

--- Rotates the ball's visual mesh to match its physical angular velocity using
--- axis-angle rotation.
--- 
--- EXECUTION FLOW:
--- 1. Check if ball has any rotation
--- 2. Calculate rotation angle for this frame: angle = angular_speed * deltaTime
--- 3. Get rotation axis (normalized direction of angular velocity)
--- 4. Create rotation CFrame using axis-angle representation
--- 5. Apply rotation to ball's current CFrame (multiply to compound rotations)
--- 
--- @param deltaTime number - Time elapsed this frame
function Ball._updateRotation(self: ClassType, deltaTime: number)
	if self.angularVelocity.Magnitude > 0 then
		-- Converts angular velocity into rotation angle for this frame
		-- Angular velocity is in radians/second, multiply by time to get radians this frame
		local rotationAngle: number = self.angularVelocity.Magnitude * deltaTime
		
		-- Get the axis of rotation (normalized direction)
		local rotationAxis: Vector3 = self.angularVelocity.Unit
		
		-- Store current orientation
		local currentCFrame: CFrame = self.instance.CFrame

		-- Create a rotation CFrame from axis and angle
		-- Negative angle accounts for the relationship between world space movement
		-- and object space rotation (right-hand rule convention)
		local rotation: CFrame = CFrame.fromAxisAngle(rotationAxis, -rotationAngle)
		
		-- Apply rotation by multiplying CFrames (order matters: current * rotation)
		-- This rotates the ball in object space, preserving its position
		self.instance.CFrame = currentCFrame * rotation
	end
end

--- Sends the ball's current CFrame to all clients via UnreliableRemoteEvent for replication.
--- Uses UnreliableRemoteEvent because occasional packet loss is acceptable for smooth visuals.
--- 
--- EXECUTION FLOW:
--- 1. Fire remote event to all clients
--- 2. Send ball ID and current CFrame
--- 3. Clients receive and update their local ball representation
function Ball._updateClients(self: ClassType)
	-- Fire to all clients with ball ID and current CFrame
	-- Clients use the ID to find their local ball instance and update its position/rotation
	UpdateEvent:FireAllClients(self.id, self.instance.CFrame)
end

--- Processes one frame of simulation by subdividing the frame delta into multiple fixed
--- substeps, ensuring physics remains stable regardless of framerate.
--- 
--- EXECUTION FLOW:
--- 1. Initialize remainingTime with the frame's deltaTime
--- 2. While there's time left to simulate:
---    a) Calculate next substep time (min of SUBSTEP_TIME or remaining time)
---    b) Call _simulate() with this substep time
---    c) Subtract substep time from remaining time
--- 3. After all substeps, check if ball fell below kill plane
--- 4. If ball is still alive, replicate position to clients
--- 
--- Example: If deltaTime = 0.025 (40 FPS) and SUBSTEP_TIME = 1/120 (0.00833):
--- - Iteration 1: simulate 0.00833s, remaining = 0.01667s
--- - Iteration 2: simulate 0.00833s, remaining = 0.00834s
--- - Iteration 3: simulate 0.00834s, remaining = 0s
--- 
--- @param deltaTime number - The time elapsed since last frame
function Ball._tick(self: ClassType, deltaTime: number)
	-- Track how much time we still need to simulate this frame
	local remainingTime: number = deltaTime

	-- Breaks variable frame time into consistent substeps for deterministic simulation
	-- This loop continues until we've simulated the entire frame duration
	while remainingTime > 0 do
		-- Take either a full substep or whatever time is left (whichever is smaller)
		local stepTime: number = math.min(SUBSTEP_TIME, remainingTime)
		
		-- Simulate physics for this substep
		self:_simulate(stepTime)
		
		-- Reduce remaining time by the amount we just simulated
		remainingTime -= stepTime
	end

	-- Automatically cleans up balls that fall below the world's kill plane
	-- FallenPartsDestroyHeight is Roblox's built-in threshold for deleting fallen parts
	if self.instance.Position.Y < workspace.FallenPartsDestroyHeight then
		self:Delete()
		return  -- Exit early since ball no longer exists
	end

	-- Replicate the ball's new position/rotation to all clients
	self:_updateClients()
end

--- Connects the ball to RunService.Heartbeat to begin running physics every frame.
--- 
--- EXECUTION FLOW:
--- 1. Create connection to Heartbeat event
--- 2. Heartbeat fires every frame with deltaTime
--- 3. Each frame calls _tick() with the deltaTime
--- 4. Store connection so it can be disconnected later
function Ball.Start(self: ClassType)
	-- Connect to Heartbeat, which fires every frame with deltaTime parameter
	self.heartbeatConnection = RunService.Heartbeat:Connect(function(deltaTime: number)
		-- Process this frame's physics simulation
		self:_tick(deltaTime)
	end)
end

--- Removes the ball from the simulation, destroys its instance, and disconnects all events.
--- 
--- EXECUTION FLOW:
--- 1. Find ball in activeBalls table
--- 2. Remove from table
--- 3. Destroy the physical instance in workspace
--- 4. Disconnect Heartbeat connection to stop physics updates
--- 5. Clean up connection reference
function Ball.Delete(self: ClassType)
	-- Find and remove this ball from the active tracking table
	for index, ball in activeBalls do
		if ball == self then
			-- Found it, remove from table
			table.remove(activeBalls, index)
			break  -- Exit loop since we found what we were looking for
		end
	end

	-- Destroy the physical part in the workspace
	self.instance:Destroy()

	-- Disconnect from Heartbeat to stop physics updates
	if self.heartbeatConnection then
		self.heartbeatConnection:Disconnect()
		self.heartbeatConnection = nil
	end
end

--- Adds external velocity to the ball and recalculates angular velocity to maintain
--- consistent rolling behavior with the new linear motion.
--- 
--- EXECUTION FLOW:
--- 1. Add the provided velocity to current velocity
--- 2. Extract horizontal component of new velocity
--- 3. Calculate horizontal speed
--- 4. If moving horizontally:
---    a) Calculate roll axis perpendicular to movement
---    b) Calculate angular velocity from linear velocity
--- 5. If not moving: zero angular velocity
--- 
--- This is typically called by external systems (e.g., when a player hits the ball)
--- 
--- @param velocity Vector3 - The
