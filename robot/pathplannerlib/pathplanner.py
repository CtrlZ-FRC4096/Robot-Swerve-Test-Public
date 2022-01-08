import wpilib
import wpimath
import json
import io
from pathlib import Path
from wpimath import geometry
from pathplannerlib.pathplanner_trajectory import PathPlannerTrajectory, Waypoint
import const

class PathPlanner( ):
	def __init__(self):
		self = self

	def loadPath(name, maxVel, maxAccel, reversed):
		# Must have .path at end of path name
        # Paths must have 3+ waypoints, no I'm not writing something to accomodate for a path that is a straight line
		trajectory = str(Path(__file__).parent.parent / "paths" / name)
		waypoints = []
		file = open(trajectory)
		data = json.load(file)
		file.close()
		for jsonWaypoint in data["waypoints"]:
			jsonAnchor = jsonWaypoint["anchorPoint"]
			anchorPoint = geometry.Translation2d(jsonAnchor["x"], jsonAnchor["y"])

			jsonPrevControl = jsonWaypoint["prevControl"]
			prevControl = None
			if jsonPrevControl != None:
				prevControl = geometry.Translation2d(jsonPrevControl["x"], jsonPrevControl["y"])

			jsonNextControl = jsonWaypoint["nextControl"]
			nextControl = None
			if jsonNextControl != None:
				nextControl = geometry.Translation2d(jsonNextControl["x"], jsonNextControl["y"])

			holonomicAngle = geometry.Rotation2d.fromDegrees(jsonWaypoint["holonomicAngle"])
			isReversal = jsonWaypoint["isReversal"]
			velOverride = -1
			if jsonWaypoint["velOverride"] != None:
				velOverride = jsonWaypoint["velOverride"]

			waypoints.append(Waypoint(anchorPoint, prevControl, nextControl, velOverride, holonomicAngle, isReversal))

		splitPaths = []
		currentPath = []

		for i in range(0, len(waypoints)):
			w = waypoints[i]
			currentPath.append(w)

			if w.isReversal or i == len(waypoints) - 1:
				splitPaths.append(currentPath)
				currentPath = []
				currentPath.append(w)

		paths = []
		shouldReverse = reversed
		for i in range(0, len(splitPaths)):
			paths.append(PathPlannerTrajectory(splitPaths[i], maxVel, maxAccel, shouldReverse))
			shouldReverse = not shouldReverse

		return PathPlanner.joinPaths(paths)

    #def loadPath(name, maxVel, maxAccel):
        #return PathPlanner.loadPath(name, maxVel, maxAccel, False)

	def joinPaths(paths):
		joinedStates = []
		for i in range(0, len(paths)):
			if i != 0:
				prevEndTime = joinedStates[i - 1].t
				for s in paths[i].getStates:
					s.t += prevEndTime

			# need to make this iterate through paths[i].getStates() and append each state to joined states
			# search for all other .get()
			#
			for path in paths:
				for state in path.states():
					joinedStates.append(state)

		return PathPlannerTrajectory(joinedStates)
