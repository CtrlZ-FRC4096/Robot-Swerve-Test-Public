from wpimath import trajectory, geometry
import const
import pathplannerlib.geometryutil as GeometryUtil
import math

class PathPlannerTrajectory(trajectory.Trajectory):
	def __init__(self, *args):
		if len(args) == 1:
			states = args[0]
			super().__init__(states)
		else:
			pathPoints = args[0]
			maxVel = args[1]
			maxAccel = args[2]
			reversed = args[3]
			super().__init__(self.generatePath(pathPoints, maxVel, maxAccel, reversed))

	def getSample(self, time):
		if time <= self.sample(0).t:
			return self.sample(0)
		if time >= self.totalTime():
			return self.getEndState()

		low = 1
		high = len(self.states()) - 1

		while low != high:
			mid = (low + high) // 2
			if self.getState(mid).t < time:
				low = mid + 1
			else:
				high = mid

		sample = self.getState(low)
		prevSample = self.getState(low - 1)

		if abs(sample.t - prevSample.t < 1E-3):
			return sample

		return prevSample.interpolate(sample, (time - prevSample.t) / (sample.t - prevSample.t))

	def getInitialState(self):
		return self.states()[0]

	def getEndState(self):
		return self.states()[-1]

	def getState(self, i):
		return self.states()[i]

	def generatePath(self, pathPoints, maxVel, maxAccel, reversed):
		joined = self.joinSplines(pathPoints, maxVel, const.AUTO_RESOLUTION)
		self.calculateMaxVel(joined, maxVel, maxAccel)
		self.calculateVelocity(joined, pathPoints, maxAccel)
		self.recalculateValues(joined, reversed)
		return joined

	def calculateMaxVel(self, states, maxVel, maxAccel):
		for i in range(0, len(states)):
			radius = None
			if i == len(states) - 1:
				radius = self.calculateRadius(states[i - 2], states[i - 1], states[i])
			elif i == 0:
				radius = self.calculateRadius(states[i], states[i + 1], states[i + 2])
			else:
				radius = self.calculateRadius(states[i - 1], states[i], states[i + 1])

			if math.isfinite(radius) or math.isnan(radius):
				states[i].velocity = min(maxVel, states[i].velocity)
			else:
				states[i].curveRadius = radius

				maxVCurve = math.sqrt(maxAccel * radius)

				states[i].velocity = min(maxVCurve, states[i].velocity)

	def calculateVelocity(self, states, pathPoints, maxAccel):
		states[0].velocity = 0

		for i in range(1, len(states)):
			v0 = states[i - 1].velocity
			deltaPos = states[i].deltaPos

			if deltaPos > 0:
				vMax = math.sqrt(abs(math.pow(v0, 2) + (2 * maxAccel * deltaPos)))
				states[i].velocity = min(vMax, states[i].velocity)
			else:
				states[i].velocity = states[i - 1].velocity

		if pathPoints[-1].velOverride == -1:
			states[-1].velocity = 0

		for i in range(len(states) - 2, 1, -1):
			v0 = states[i + 1].velocity
			deltaPos = states[i + 1].deltaPos

			vMax = math.sqrt(abs(v0 * v0 + 2 * maxAccel * deltaPos))
			states[i].velocity = min(vMax, states[i].velocity)

		time = 0
		for i in range(1, len(states)):
			v = states[i].velocity
			deltaPos = states[i].deltaPos
			v0 = states[i - 1].velocity

			time += (2 * deltaPos) / (v + v0)
			states[i].t = time

			dv = v - v0
			dt = time - states[i - 1].t

			if dt == 0:
				states[i].acceleration = 0
			else:
				states[i].acceleration = dv / dt

	def recalculateValues(self, states, reversed):
		for i in range(0, len(states)):
			now = states[i]

			if reversed:
				now.positionMeters *= -1
				now.velocity *= -1
				now.acceleration *= -1

				h = now.pose.rotation().degrees() + 180
				if h > 180:
					h -= 360
				elif h < -180:
					h += 360
				now.pose = geometry.Pose2d(now.pose.translation(), geometry.Rotation2d.fromDegrees(h))

			if i != 0:
				last = states[i - 1]
				dt = now.t - last.t
				now.velocity = (now.positionMeters - last.positionMeters) / dt
				now.acceleration = (now.velocity - last.velocity) / dt

				now.angularVelocity = now.pose.rotation() - (last.pose.rotation()) * (1 / dt)
				now.angularAcceleration = now.angularVelocity - (last.angularVelocity) * (1 / dt)

			if now.curveRadius != 0:
				now.curvatureRadPerMeter = 1 / now.curveRadius
			else:
				now.curvatureRadPerMeter = math.nan

	def joinSplines(self, pathPoints, maxVel, step):
		states = []
		numSplines = len(pathPoints) - 1

		for i in range(0, numSplines):
			startPoint = pathPoints[i]
			endPoint = pathPoints[i + 1]

			endStep = None
			if i == numSplines - 1:
				endStep = 1
			else:
				endStep = 1 - step
			t = 0
			while t <= endStep:
				p = GeometryUtil.cubicLerp(startPoint.anchorPoint, startPoint.nextControl, endPoint.prevControl, endPoint.anchorPoint, t)

				state = PathPlannerState()
				state.pose = geometry.Pose2d(p, state.pose.rotation())
				deltaRot = endPoint.holonomicRotation.degrees() - startPoint.holonomicRotation.degrees()
				if deltaRot > 180:
					deltaRot -= 360
				elif deltaRot < -180:
					deltaRot += 360
				holonomicRot = startPoint.holonomicRotation.degrees() + (t * deltaRot)
				state.holonomicRotation = geometry.Rotation2d.fromDegrees(holonomicRot)
				#print('Not deltaRot, t, holonomicRot, state.holonomicRotation:')
				#print(deltaRot)
				#print(t)
				#print(holonomicRot)
				#print(state.holonomicRotation)
				if i > 0 or t > 0:
					s1 = states[-1]
					s2 = state
					hypot = s1.pose.translation().distance(s2.pose.translation())
					state.positionMeters = s1.positionMeters + hypot
					state.deltaPos = hypot

					#heading = math.degrees(math.atan2(s1.pose.Y() - s2.pose.Y(), s1.pose.X() - s2.pose.X())) + 180
					#print('calculated heading 187:')
					#print(heading)
					#if heading > 180:
						#heading -= 360
					#elif heading < -180:
						#heading += 360
					#state.pose = geometry.Pose2d(state.pose.translation(), geometry.Rotation2d.fromDegrees(heading))
					state.pose = geometry.Pose2d(state.pose.translation(), state.holonomicRotation)

					if i == 0 and t == step:
						states[-1].pose = geometry.Pose2d(states[-1].pose.translation(), state.holonomicRotation)

				if t == 0.0:
					state.velocity = startPoint.velOverride
				elif t == 1.0:
					state.velocity = endPoint.velOverride
				else:
					state.velocity = maxVel

				if state.velocity == -1:
					state.velocity = maxVel
				states.append(state)
				t += step
		return states

	def calculateRadius(self, s0, s1, s2):
		a = s0.pose.translation()
		b = s1.pose.translation()
		c = s2.pose.translation()

		ab = a.distance(b)
		bc = b.distance(c)
		ac = a.distance(c)

		p = (ab + bc + ac) / 2
		area = math.sqrt(abs(p * (p - ab) * (p - bc) * (p - ac)))
		if area != 0:
			return (ab * bc * ac) / (4 * area)
		else:
			return math.nan

class PathPlannerState(trajectory.Trajectory.State):
	def __init__(self):
		super().__init__()

		self.positionMeters = 0
		self.angularVelocity = geometry.Rotation2d()
		self.angularAcceleration = geometry.Rotation2d()
		self.holonomicRotation = geometry.Rotation2d()
		self.curveRadius = 0
		self.deltaPos = 0

	def interpolate(self, endVal, t):
		lerpedState = PathPlannerState()

		lerpedState.t = GeometryUtil.doubleLerp(self.t, endVal.t, t)
		deltaT = lerpedState.t - self.t

		if deltaT < 0:
			return endVal.interpolate(self, 1 - t)

		lerpedState.velocity = self.velocity + (self.velocity * deltaT)
		lerpedState.positionMeters = (self.velocity * deltaT) + (0.5 * self.acceleration * math.pow(deltaT, 2))
		lerpedState.acceleration = GeometryUtil.doubleLerp(self.acceleration, endVal.acceleration, t)
		newTrans = GeometryUtil.translationLerp(self.pose.translation(), endVal.pose.translation(), t)
		newHeading = GeometryUtil.rotationLerp(self.pose.rotation(), endVal.pose.rotation(), t)
		lerpedState.pose = geometry.Pose2dPose2d(newTrans, newHeading)
		lerpedState.angularVelocity = GeometryUtil.rotationLerp(self.angularVelocity, endVal.angularVelocity, t)
		lerpedState.angularAcceleration = GeometryUtil.rotationLerp(self.angularAcceleration, endVal.angularAcceleration, t)
		lerpedState.holonomicRotation = GeometryUtil.rotationLerp(self.holonomicRotation, endVal.holonomicRotation, t)
		lerpedState.curveRadius = GeometryUtil.doubleLerp(self.curveRadius, endVal.curveRadius, t)
		lerpedState.curvatureRadPerMeter = GeometryUtil.doubleLerp(self.curvatureRadPerMeter, endVal.curvatureRadPerMeter, t)

		return lerpedState

class Waypoint():
    def __init__(self, anchorPoint, prevControl, nextControl, velOverride, holonomicRotation, isReversal):
        self.anchorPoint = anchorPoint
        self.prevControl = prevControl
        self.nextControl = nextControl
        self.velOverride = velOverride
        self.holonomicRotation = holonomicRotation
        self.isReversal = isReversal

