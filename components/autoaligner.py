from networktables import NetworkTables

from wpilib_controller import PIDController

from components.drivetrain import Drivetrain

from magicbot import tunable

import navx


class AutoAligner:

    CHECKS = 5
    K = tunable(default=0.2)
    Y_SPEED = tunable(default=0.2)
    MAX_SPEED = tunable(default=0.6)

    drivetrain: Drivetrain
    navx: navx.ahrs.AHRS

    def __init__(self):

        self.vision = NetworkTables.getTable('Vision')
        self.enabled = False

    def setup(self):

        self.navx_controller = PIDController(
            0.015, 0, 0, measurement_source=self.get_navx_error)

        self.navx_controller.setInputRange(-180, 180)
        self.navx_controller.setOutputRange(-self.MAX_SPEED, self.MAX_SPEED)
        self.navx_controller.setContinuous()
        self.navx_controller.setReference(-90)
        self.navx_controller.setPercentTolerance(0.01)

        self.angle_controller = PIDController(
            0.1, 0, 0, measurement_source=self.get_angle_error)

        self.angle_controller.setInputRange(-31.1, 31.1)
        self.angle_controller.setOutputRange(-self.MAX_SPEED, self.MAX_SPEED)
        self.angle_controller.setReference(0)
        self.angle_controller.setPercentTolerance(0.01)

    def get_angle_error(self):

        return -self.vision.getNumber('X Angle', 0)

    def get_navx_error(self):

        return self.navx.getYaw()

    def enable(self):
        self.enabled = True

    def reset(self):
        self.angle_controller.reset()
        self.navx_controller.reset()
        self.enabled = False
        self.positioned = 0
        self.drivetrain.unlock()

    def execute(self):

        if self.enabled:
            self.drivetrain.lock()
            angle_output = self.angle_controller.update()
            self.vision.putNumber("AnglePID Output", angle_output)

            navx_output = self.navx_controller.update()
            self.vision.putNumber("NavxPID Output", navx_output)

            self.drivetrain.move(
                self.K
                * angle_output
                + (self.MAX_SPEED - self.K)
                * navx_output,
                self.Y_SPEED
            )

            # if self.x_controller.atReference():
            #     self.positioned += 1
            # else:
            #     self.positioned = 0

            # if self.positioned == self.CHECKS:
            #     self.reset()
