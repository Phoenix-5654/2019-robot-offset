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


        self.y_controller = PIDController(
            1, 0, 0, measurement_source=self.get_y_error)

        self.y_controller.setInputRange(-160, 160)
        self.y_controller.setOutputRange(-1, 1)
        self.y_controller.setReference(0)
        self.y_controller.setPercentTolerance(0.01)

        self.y_enabled = False

        self.positioned = 0

    def get_x_error(self):

        return self.vision.getNumber('X Error', 0)

    def get_y_error(self):

        return self.vision.getNumber('Y Error', 0)

    def enable_x(self):
        self.x_enabled = True

    def enable_y(self):
        self.y_enabled = True

    def enable(self):
        self.enable_x()
        self.enable_y()
    def reset(self):
        self.x_controller.reset()
        self.y_controller.reset()
        self.x_enabled = False
        self.y_enabled = False
        self.positioned = 0
        self.drivetrain.unlock()
    def execute(self):

        if self.x_enabled or self.y_enabled:
            self.drivetrain.lock()
            x_output = self.x_controller.update()
            y_output = self.y_controller.update()
            self.drivetrain.move(x_output, -y_output)

            if self.x_controller.atReference() and self.y_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.reset()

