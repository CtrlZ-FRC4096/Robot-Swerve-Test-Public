#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

DEBUG = True

# Import our files
import logging
import time
import sys

import wpilib
import commands2

import networktables

import const
import oi

import subsystems.drivetrain
import commands.autonomous

log = logging.getLogger('robot')
networktables.NetworkTables.initialize()



class Robot(commands2.TimedCommandRobot):
	"""
	Main robot class.

	This is the central object, holding instances of all the robot subsystem
	and sensor classes.

	It also contains the init & periodic methods for autonomous and
	teloperated modes, called during mode changes and repeatedly when those
	modes are active.

	The one instance of this class is also passed as an argument to the
	various other classes, so they have full access to all its properties.
	"""
	def robotInit(self):

		self.match_time = -1
		const.IS_SIMULATION = self.isSimulation( )

		# NetworkTables
		self.nt_robot = networktables.NetworkTables.getTable('Robot')

		### Subsystems ###

		self.drivetrain = subsystems.drivetrain.Drivetrain(self)

		self.subsystems = [
			self.drivetrain,
		]

		### OTHER ###

		# Driverstation
		self.driverstation : wpilib.DriverStation = wpilib.DriverStation.getInstance()

		# Operator Input
		self.oi = oi.OI(self)

		## Scheduler ##
		self.scheduler = commands2.CommandScheduler.getInstance()


		### LOGGING ###

		# Timers for NetworkTables update so we don't use too much bandwidth
		self.log_timer = wpilib.Timer()
		self.log_timer.start()
		self.log_timer_delay = 0.25		# 4 times/second

		# Disable LW telemetry before comp to improve loop dtimes
		# wpilib.LiveWindow.disableAllTelemetry()

		self.match_time = -1


	def robotPeriodic(self):
		# Stuff for the wpilib simulator
		self.log()
		self.oi.log()


	### DISABLED ###

	def disabledInit(self):
		self.scheduler.cancelAll()

		# for subsystem in self.subsystems:
		# 	subsystem.stop()


	def disabledPeriodic(self):
		pass


	### AUTONOMOUS ###

	def autonomousInit(self):
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Test10Ft.path')
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Test10Ft180.path')
		self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'Slalom.path')
		#self.selected_auto_mode = commands.autonomous.DriveTrajectory(self, 'WiggleReverse.path')
		self.scheduler.schedule(self.selected_auto_mode)

	def autonomousPeriodic(self):
		self.scheduler.run()


	### TELEOPERATED ###

	def teleopInit(self):
		# Removes any leftover commands from the scheduler
		self.scheduler.cancelAll()


	def teleopPeriodic(self):
		self.scheduler.run()


	### MISC ###

	def log(self):
		"""
		Logs some info to shuffleboard, and standard output
		"""
		if not self.log_timer.advanceIfElapsed(self.log_timer_delay):
			return

		#self.nt_robot.putString('Pressure', '{0:.2f}'.format(self.get_pressure()))

		for s in self.subsystems:
			s.log()

		self.match_time = self.driverstation.getMatchTime()

### MAIN ###

if __name__ == "__main__":
	wpilib.run(Robot)
