from networktables import NetworkTables

from wpilib_controller import PIDController
from navx import AHRS

from components.drivetrain import Drivetrain


class AutoAligner:
    CHECKS = 5

    drivetrain: Drivetrain
    navx: AHRS

    def __init__(self):

        self.vision = NetworkTables.getTable('Vision')

        self.angle_controller = PIDController(
            0.015, 0, 0, measurement_source=self.get_angle)
        self.angle_controller.setInputRange(-180, 180)
        self.angle_controller.setOutputRange(-0.8, 0.8)
        self.angle_controller.setReference(0)
        self.angle_controller.setAbsoluteTolerance(3)
        self.angle_controller.setContinuous()

        self.angle_controller_enabled = False

        self.y_controller = PIDController(
            0.003, 0, 0, measurement_source=self.get_y_error)

        self.y_controller.setInputRange(-120, 120)
        self.y_controller.setOutputRange(-0.6, 0.6)
        self.y_controller.setReference(0)
        self.y_controller.setAbsoluteTolerance(15)

        self.y_enabled = False

        self.positioned = 0

    def setup(self):
        self.navx.reset()


    def get_angle(self):
        return self.navx.getYaw()

    def get_y_error(self):
        return self.vision.getNumber('Y Error', 0)

    def enable_angle(self):
        self.angle_controller_enabled = True

    def enable_y(self):
        self.y_enabled = True

    def enable(self):
        self.enable_angle()
        self.enable_y()

    def reset(self, reset_gyro=False):
        self.angle_controller.reset()
        self.y_controller.reset()
        self.angle_controller_enabled = False
        self.y_enabled = False
        self.positioned = 0
        self.drivetrain.unlock()
        if reset_gyro:
            self.navx.reset()

    def execute(self):

        if self.angle_controller_enabled or self.y_enabled:
            self.drivetrain.lock()
            angle_output = self.angle_controller.update()
            y_output = self.y_controller.update()
            self.drivetrain.move(angle_output, -y_output, False)

            if self.angle_controller.atReference() and self.y_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.reset()

        self.vision.putNumber("Angle", self.get_angle())
