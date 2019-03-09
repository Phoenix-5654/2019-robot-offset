from magicbot import tunable
from networktables import NetworkTables

from components.drivetrain import Drivetrain
from components.wpilib_controller import PIDController


class AutoAligner:
    MAX_X_SPEED = tunable(default=1)
    HORIZONTAL_RES = 240
    VERTICAL_RES = 320

    drivetrain: Drivetrain

    def __init__(self):
        self.vision = NetworkTables.getTable('Vision')
        self.enabled = False
        self.y_output = 0

    def setup(self):
        self.x_controller = PIDController(
            0.005, 0, 0, measurement_source=self.get_x_error)

        self.x_controller.setInputRange(-self.VERTICAL_RES, self.VERTICAL_RES)
        self.x_controller.setOutputRange(-self.MAX_X_SPEED,
                                         self.MAX_X_SPEED)
        self.x_controller.setAbsoluteTolerance(10)
        self.x_controller.setName("X controller")

        self.reset()

    def get_x_error(self):
        return self.vision.getNumber('X Error', 0)

    def set_y(self, y):
        self.y_output = y

    def enable(self):
        self.enabled = True
        self.drivetrain.lock()

    def reset(self):
        self.x_controller.reset()
        self.drivetrain.unlock()
        self.enabled = False

    def execute(self):
        if self.enabled:
            x_output = self.x_controller.update()
            self.vision.putNumber("X-PID Output", x_output)
            self.drivetrain.move(
                x_output,
                self.y_output
            )
