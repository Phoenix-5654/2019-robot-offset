from networktables import NetworkTables

from wpilib_controller import PIDController

from components.drivetrain import Drivetrain


class AutoAligner:

    CHECKS = 5

    drivetrain: Drivetrain

    def __init__(self):

        self.vision = NetworkTables.getTable('Vision')

        self.x_controller = PIDController(
            1, 0, 0, measurement_source=self.get_x_error)

        self.x_controller.setInputRange(-160, 160)
        self.x_controller.setOutputRange(-1, 1)
        self.x_controller.setReference(0)
        self.x_controller.setPercentTolerance(0.01)

        self.x_enabled = False

        self.positioned = 0

    def get_x_error(self):

        return self.vision.getNumber('X Error', 0)

    def get_y_error(self):

        return self.vision.getNumber('Y Error', 0)

    def enable_x(self):
        self.x_enabled = True

    def execute(self):

        if self.x_enabled:
            self.drivetrain.lock()
            output = self.x_controller.update()
            self.drivetrain.move(output, 0)

            if self.x_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.x_controller.reset()
                self.x_enabled = False
                self.positioned = 0
                self.drivetrain.unlock()
