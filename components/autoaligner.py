from networktables import NetworkTables

from components.wpilib_controller import PIDController

from components.drivetrain import Drivetrain

from magicbot import tunable

import navx


class AutoAligner:

    CHECKS = 10
    Y_SPEED = tunable(default=0.2)
    MAX_SPEED_NAVX = tunable(default=0.8)
    MAX_SPEED_STRIGHT = tunable(default=0.8)
    ERROR = 5000

    drivetrain: Drivetrain
    navx: navx.ahrs.AHRS

    def __init__(self):
        self.first_state = self.second_state = self.third_state = self.fourth_state = False
        self.vision = NetworkTables.getTable('Vision')
        self.enabled = False

    def setup(self):

        self.navx_controller = PIDController(
            0.025, 0.01, 0, measurement_source=self.get_navx_error)

        self.navx_controller.setInputRange(-180, 180)
        self.navx_controller.setOutputRange(-self.MAX_SPEED_NAVX,
                                            self.MAX_SPEED_NAVX)
        self.navx_controller.setContinuous()
        self.navx_controller.setAbsoluteTolerance(3.5)

        self.x_controller = PIDController(
            0.1, 0, 0, measurement_source=self.get_x_error)

        self.x_controller.setInputRange(-160, 160)
        self.x_controller.setOutputRange(-self.MAX_SPEED_STRIGHT,
                                         self.MAX_SPEED_STRIGHT)
        self.x_controller.setPercentTolerance(0.01)

        self.reset()

    def get_x_error(self):

        return self.vision.getNumber('X Error', 0)

    def get_navx_error(self):

        return self.navx.getYaw()

    def enable(self):
        self.state = 1
        self.navx_controller.setReference(0)

    def reset(self):
        self.x_controller.reset()
        self.navx_controller.reset()
        self.state = 0
        self.positioned = 0
        self.drivetrain.unlock()

    def execute(self):
        self.vision.putNumber("State", self.state)

        if self.state == 1:
            self.drivetrain.lock()
            navx_output = self.navx_controller.update()
            self.vision.putNumber("NavxPID Output", navx_output)

            self.drivetrain.move(
                navx_output,
                0
            )

            if self.navx_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.state += 1
                self.x_controller.setReference(0)
                self.positioned = 0

        if self.state == 2:
            if self.get_x_error() == self.ERROR:
                self.drivetrain.move(
                    0,
                    self.Y_SPEED
                )
            else:
                x_output = self.x_controller.update()
                self.vision.putNumber("XPID Output", x_output)

                self.drivetrain.move(
                    0,
                    x_output
                )

                if self.navx_controller.atReference():
                    self.positioned += 1
                else:
                    self.positioned = 0

                if self.positioned == self.CHECKS:
                    self.state += 1
                    self.navx_controller.setReference(-90)
                    self.positioned = 0

        if self.state == 3:
            navx_output = self.navx_controller.update()
            self.vision.putNumber("NavxPID Output", navx_output)

            self.drivetrain.move(
                navx_output,
                0
            )

            if self.navx_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.state += 1
                self.positioned = 0

        if self.state == 4:

            self.drivetrain.move(
                0,
                self.Y_SPEED
            )
