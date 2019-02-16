import ctre
import wpilib
import wpilib.drive
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
from components.autoaligner import AutoAligner


class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    auto_aligner: AutoAligner

    def createObjects(self):

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

        self.joystick = wpilib.Joystick(0)

        self.controller = wpilib.XboxController(1)

    def teleopPeriodic(self):

        if self.controller.getStartButtonPressed():
            self.auto_aligner.enable_x()

        if not self.drivetrain.locked:
            self.drivetrain.move(self.joystick.getX(), -self.joystick.getY())


if __name__ == "__main__":
    wpilib.run(MyRobot)
