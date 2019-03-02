import math
import os
import csv

import ctre
import navx
import pathfinder as pf
from magicbot import AutonomousStateMachine, state, tunable, feedback

from autonomous.path_generator import PathGenerator
from components.drivetrain import Drivetrain
from components.wpilib_controller import PIDController
from networktables import NetworkTables


class RightAuto(AutonomousStateMachine):
    drivetrain: Drivetrain
    auto_left_motor: ctre.WPI_TalonSRX
    auto_right_motor: ctre.WPI_TalonSRX
    navx: navx.ahrs.AHRS

    MODE_NAME = "Right Auto"
    DEFAULT = True
    # points = [pf.Waypoint(2, 5.25, 0),
    #           pf.Waypoint(3.75, 5.25, 0),
    #           pf.Waypoint(5.815, 5.9, 0),
    #           pf.Waypoint(6.9, 4.95, math.radians(90))]
    points = [pf.Waypoint(0, 0, 0),
              pf.Waypoint(3, 0, 0)]

    RESOLUTION = 1024
    WHEEL_DIAMETER = 0.1524

    MAX_LINEAR_SPEED = tunable(default=0.7)
    MAX_ANGULAR_SPEED = tunable(default=0)#0.35



    def setup(self):
        self.tab = NetworkTables.getTable("testing output")
        print(os.path.join(os.path.dirname(__file__)
                , "../auto_right_right_Jaci.csv"))
        with open(
                os.path.join(os.path.dirname(__file__)
                    , "../auto_right_right_Jaci.csv"), "r") as f:
            reader = csv.DictReader(f)
            self.right_position = []
            self.heading = []
            for row in reader:  # read a row as {column1: value1, column2: value2,...}
                self.right_position.append(float(row["position"]))
                self.heading.append(math.degrees(float(row["heading"])))

        self.heading.pop()
        with open(
                os.path.join(os.path.dirname(__file__)
                    , "../auto_right_left_Jaci.csv"), "r") as f:
            reader = csv.DictReader(f)
            self.left_position = []
            for row in reader:
                self.left_position.append(float(row["position"]))

        self.right_encoder_controller = PIDController(
            1, 0, 0, measurement_source=self.get_right)

        self.right_encoder_controller.setInputRange(-2.5, 2.5)
        self.right_encoder_controller.setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)
        self.right_encoder_controller.setAbsoluteTolerance(0)
        self.right_encoder_controller.setName("Right encoder controller")

        self.left_encoder_controller = PIDController(
            1, 0, 0, measurement_source=self.get_left)

        self.left_encoder_controller .setInputRange(-2.5, 2.5)
        self.left_encoder_controller .setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)
        self.left_encoder_controller .setAbsoluteTolerance(0)
        self.left_encoder_controller .setName("Left encoder controller")

        self.gyro_controller = PIDController(
            1, 0, 0, measurement_source=self.get_yaw)

        self.gyro_controller .setInputRange(-180, 180)
        self.gyro_controller.setOutputRange(-self.MAX_ANGULAR_SPEED,
                                         self.MAX_ANGULAR_SPEED)
        self.gyro_controller.setAbsoluteTolerance(0)
        self.gyro_controller.setContinuous()
        self.gyro_controller.setName("Gyro controller")

        self.last_right_encoder_value = 0
        self.last_left_encoder_value = 0
        self.counter = 0

    def get_right(self):
        self.right_pos = ((self.auto_right_motor.getQuadraturePosition()
                / self.RESOLUTION) * self.WHEEL_DIAMETER)
        return self.right_pos

    def get_left(self):
        self.left_pos = -((self.auto_left_motor.getQuadraturePosition()
                / self.RESOLUTION) * self.WHEEL_DIAMETER)
        return self.left_pos

    def get_yaw(self):
        self.angle = self.navx.getYaw()
        return self.angle

    @state(first=True)
    def reset(self):
        self.auto_right_motor.setQuadraturePosition(0, 10)
        self.auto_left_motor.setQuadraturePosition(0, 10)
        self.navx.reset()
        self.counter = 0
        self.last_right_encoder_value = 0
        self.last_left_encoder_value = 0

        self.left_encoder_controller .setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)

        self.right_encoder_controller .setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)
        self.gyro_controller.setOutputRange(-self.MAX_ANGULAR_SPEED,
                                                     self.MAX_ANGULAR_SPEED)
        self.next_state("execute_auto")




    @state(must_finish=True)
    def execute_auto(self):
        self.left_encoder_controller.setReference(self.left_position[self.counter])
        self.left_output = self.left_encoder_controller.update()
        self.right_encoder_controller.setReference(self.right_position[self.counter])
        self.right_output = self.right_encoder_controller.update()
        self.gyro_controller.setReference(self.heading[self.counter])
        self.gyro_output = self.gyro_controller.update()

        self.printer()

        self.drivetrain.tank_move(self.left_output + self.gyro_output,
                                  self.right_output - self.gyro_output)
        self.counter += 1
        if self.counter == len(self.heading):
            self.done()

    def printer(self):
        self.tab.putNumber("counter", self.counter)

        self.tab.putNumber("Right Position", self.right_pos)
        self.tab.putNumber("Right Setpoint", self.right_position[self.counter])
        self.tab.putNumber("RightPID output", self.right_output)

        self.tab.putNumber("Left Position", self.left_pos)
        self.tab.putNumber("Left Setpoint", self.left_position[self.counter])
        self.tab.putNumber("LeftPID output", self.left_output)

        self.tab.putNumber("gyro Position", self.angle)
        self.tab.putNumber("gyro Setpoint", self.heading[self.counter])
        self.tab.putNumber("gyroPID output", self.gyro_output)

