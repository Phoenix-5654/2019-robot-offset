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
            self.right_velocity = []
            self.angular_velocity = [0]
            for row in reader:  # read a row as {column1: value1, column2: value2,...}
                self.right_velocity.append(float(row["velocity"]))
                self.angular_velocity.append(math.degrees(float(row["heading"])))

        for i in range(len(self.angular_velocity)):
            if i == 0:
                continue
            self.angular_velocity[i - 1] = (self.angular_velocity[i]
                                           - self.angular_velocity[i - 1]) / 0.02

        self.angular_velocity.pop()
        with open(
                os.path.join(os.path.dirname(__file__)
                    , "../auto_right_left_Jaci.csv"), "r") as f:
            reader = csv.DictReader(f)
            self.left_velocity = []
            for row in reader:
                self.left_velocity.append(float(row["velocity"]))

        self.right_encoder_controller = PIDController(
            1, 0, 0, measurement_source=self.get_right_velocity)

        self.right_encoder_controller.setInputRange(-2.5, 2.5)
        self.right_encoder_controller.setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)
        self.right_encoder_controller.setAbsoluteTolerance(0)
        self.right_encoder_controller.setName("Right encoder controller")

        self.left_encoder_controller = PIDController(
            1, 0, 0, measurement_source=self.get_left_velocity)

        self.left_encoder_controller .setInputRange(-2.5, 2.5)
        self.left_encoder_controller .setOutputRange(-self.MAX_LINEAR_SPEED,
                                         self.MAX_LINEAR_SPEED)
        self.left_encoder_controller .setAbsoluteTolerance(0)
        self.left_encoder_controller .setName("Left encoder controller")

        self.gyro_controller = PIDController(
            1, 0, 0, measurement_source=self.get_angular_velocity)

        self.gyro_controller .setInputRange(-90, 90)
        self.gyro_controller.setOutputRange(-self.MAX_ANGULAR_SPEED,
                                         self.MAX_ANGULAR_SPEED)
        self.gyro_controller.setAbsoluteTolerance(0)
        self.gyro_controller.setName("Gyro controller")

        self.last_right_encoder_value = 0
        self.last_left_encoder_value = 0
        self.counter = 0

    @feedback(key="right_velocity")
    def get_right_velocity(self):
        dist = ((self.auto_right_motor.getQuadraturePosition()
                / self.RESOLUTION) * self.WHEEL_DIAMETER)
        vel = (dist - self.last_right_encoder_value) / 0.02
        self.last_left_encoder_value = dist
        return vel

    @feedback
    def get_left_velocity(self):
        dist = ((self.auto_left_motor.getQuadraturePosition()
                / self.RESOLUTION) * self.WHEEL_DIAMETER)
        vel = (dist - self.last_left_encoder_value) / 0.02
        self.last_left_encoder_value = dist
        return vel

    @feedback
    def get_angular_velocity(self):
        return self.navx.getRate()

    @state(first=True)
    def reset(self):
        self.auto_right_motor.setQuadraturePosition(0, 10)
        self.auto_left_motor.setQuadraturePosition(0, 10)
        self.navx.reset()
        self.counter = 0
        self.last_right_encoder_value = 0
        self.last_left_encoder_value = 0
        self.next_state("execute_auto")

    @state(must_finish=True)
    def execute_auto(self):
        self.printer()
        self.left_encoder_controller.setReference(self.left_velocity[self.counter])
        left_output = self.left_encoder_controller.update()
        self.right_encoder_controller.setReference(self.right_velocity[self.counter])
        right_output = self.right_encoder_controller.update()
        self.gyro_controller.setReference(self.angular_velocity[self.counter])
        gyro_output = self.gyro_controller.update()

        self.counter += 1

        self.drivetrain.tank_move(left_output + gyro_output,
                                  right_output - gyro_output)

        if self.counter == len(self.angular_velocity):
            self.done()

    def printer(self):
        self.tab.putNumber("counter", self.counter)
        self.tab.putNumber("right vel diff", self.get_right_velocity())
        self.tab.putNumber("left vel diff", self.get_left_velocity())
        self.tab.putNumber("angular vel diff", self.get_angular_velocity())
        self.tab.putNumber("angular vel", self.navx.getRate())
