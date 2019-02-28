from networktables import NetworkTables

from components.wpilib_controller import PIDController

from components.drivetrain import Drivetrain

from magicbot import tunable


class AutoAligner:
    CHECKS = 1

    MAX_Y_SPEED = tunable(default=1)
    MAX_X_SPEED = tunable(default=1)
    HORIZONTAL_RES = 240
    VERTICAL_RES = 320

    drivetrain: Drivetrain

    def __init__(self):
        self.vision = NetworkTables.getTable('Vision')
        self.enabled = False

    def setup(self):

        self.y_controller = PIDController(
            1, 0, 0, measurement_source=self.get_y_error)

        self.y_controller.setInputRange(-self.HORIZONTAL_RES //
                                        2, self.HORIZONTAL_RES // 2)
        self.y_controller.setOutputRange(-self.MAX_Y_SPEED,
                                         self.MAX_Y_SPEED)
        self.y_controller.setAbsoluteTolerance(10)
        self.y_controller.setName("Y controller")

        self.x_controller = PIDController(
            1, 0, 0, measurement_source=self.get_x_error)

        self.x_controller.setInputRange(-self.VERTICAL_RES, self.VERTICAL_RES)
        self.x_controller.setOutputRange(-self.MAX_X_SPEED,
                                         self.MAX_X_SPEED)
        self.x_controller.setAbsoluteTolerance(10)
        self.x_controller.setName("X controller")

        self.reset()

    def get_x_error(self):

        return self.vision.getNumber('X Error', 0)

    def get_y_error(self):

        return self.vision.getNumber('Y Error', 0)

    def enable(self):
        self.enabled = True

    def reset(self):
        self.x_controller.reset()
        self.y_controller.reset()
        self.positioned = 0
        self.drivetrain.unlock()
        self.enabled = False

    def execute(self):
        if self.enabled:
            self.drivetrain.lock()
            x_output = self.x_controller.update()
            y_output = self.y_controller.update()
            self.vision.putNumber("XPID Output", x_output)
            self.vision.putNumber("YPID Output", y_output)
            self.drivetrain.move(
                x_output,
                -y_output
            )
            if self.y_controller.atReference() and self.x_controller.atReference():
                self.positioned += 1
            else:
                self.positioned = 0

            if self.positioned == self.CHECKS:
                self.positioned = 0
                self.reset()
