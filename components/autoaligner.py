import wpilib
from magicbot import tunable
from networktables import NetworkTables

from components.drivetrain import Drivetrain
from components.wpilib_controller import PIDController


class AutoAligner:
    MAX_X_SPEED = tunable(default=1)
    MAX_RANGE_SPEED = tunable(default=0.6)
    HORIZONTAL_RES = 240
    VERTICAL_RES = 320

    RANGE_REFERENCE = tunable(default=50)
    CHECKS = 3

    drivetrain: Drivetrain
    range_sensor: wpilib.Ultrasonic

    def __init__(self):
        self.vision = NetworkTables.getTable('Vision')
        self.enabled = self.position_enabled = False
        self.y_output = 0
        self.checks = 0

    def setup(self):
        self.x_controller = PIDController(
            0.005, 0, 0, measurement_source=self.get_x_error)

        self.x_controller.setInputRange(-self.VERTICAL_RES, self.VERTICAL_RES)
        self.x_controller.setOutputRange(-self.MAX_X_SPEED,
                                         self.MAX_X_SPEED)
        self.x_controller.setAbsoluteTolerance(10)
        self.x_controller.setName("X Controller")

        self.range_controller = PIDController(
            0.1, 0, 0, measurement_source=self.get_range)

        self.range_controller.setInputRange(0, 634)
        self.range_controller.setOutputRange(-self.MAX_RANGE_SPEED,
                                             self.MAX_RANGE_SPEED)
        self.range_controller.setAbsoluteTolerance(25)
        self.range_controller.setName("Range Controller")

        self.reset()

    def get_x_error(self):
        return self.vision.getNumber('X Error', 0)

    def get_range(self):
        return self.range_sensor.getRangeMM() / 10

    def set_y(self, y):
        self.y_output = y

    def enable(self):
        self.enabled = True
        self.drivetrain.lock()

    def get_to_position(self):
        self.position_enabled = True
        self.range_controller.setReference(self.RANGE_REFERENCE)
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

        if self.position_enabled:
            range_output = self.range_controller.update()
            self.vision.putNumber("Range Output", range_output)
            self.drivetrain.move(
                0,
                range_output
            )

            if self.checks == self.CHECKS:
                self.range_controller.reset()
                self.position_enabled = False
            elif self.range_controller.atReference():
                self.checks += 1
            else:
                self.checks = 0

        self.vision.putNumber(
            "Range Sensor [cm]",
            self.get_range()
        )
