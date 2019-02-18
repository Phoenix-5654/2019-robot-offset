import ctre
import wpilib
import wpilib.drive
from magicbot import MagicRobot
import navx

from components.drivetrain import Drivetrain
from components.autoaligner import AutoAligner


class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    auto_aligner: AutoAligner

    def createObjects(self):

        self.navx = navx.ahrs.AHRS.create_i2c()

        self.left_front_motor = ctre.WPI_TalonSRX(1)
        self.left_rear_motor = ctre.WPI_VictorSPX(2)

        self.right_front_motor = ctre.WPI_TalonSRX(3)
        self.right_rear_motor = ctre.WPI_VictorSPX(4)

        self.left = wpilib.SpeedControllerGroup(
            self.left_front_motor, self.left_rear_motor
        )

        self.right = wpilib.SpeedControllerGroup(
            self.right_front_motor, self.right_rear_motor
        )

        self.drive = wpilib.drive.DifferentialDrive(
            self.left, self.right
        )

        self.right_joystick = wpilib.Joystick(0)
        self.left_joystick = wpilib.Joystick(1)

        self.controller = wpilib.XboxController(2)
    def testInit(self):
        self.auto_aligner.reset(True)

    def teleopInit(self):
        self.auto_aligner.reset(True)

    def teleopPeriodic(self):

        if self.controller.getStartButtonPressed():
            self.auto_aligner.enable()

        if not self.drivetrain.locked:
            self.drivetrain.tank_move(-self.left_joystick.getY(),
                                      -self.right_joystick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)
