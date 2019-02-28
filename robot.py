import ctre
import navx
import wpilib
import wpilib.drive
from magicbot import MagicRobot
from networktables import NetworkTables

from components.autoaligner import AutoAligner
from components.drivetrain import Drivetrain
from components.grippers import Grippers
from components.piston import Piston
from components.motor import Motor, MotorConfig
from components.single_solenoid_piston import SingleSolenoidPiston

from autonomous.right_auto import RightAuto


class MyRobot(MagicRobot):
    drivetrain: Drivetrain
    auto_aligner: AutoAligner
    grippers: Grippers
    hatch_panel_piston: Piston
    gripper_piston: Piston
    panel_mechanism_piston: Piston
    first_hatch_panel_piston: SingleSolenoidPiston
    ramp: Motor

    RIGHT_CONTROLLER_HAND = wpilib.XboxController.Hand.kRight
    LEFT_CONTROLLER_HAND = wpilib.XboxController.Hand.kLeft

    def createObjects(self):
        NetworkTables.initialize()

        wpilib.CameraServer.launch()

        self.tab = NetworkTables.getTable('Navx')

        self.auto_left_motor = self.left_front_motor = ctre.WPI_TalonSRX(1)
        self.left_rear_motor = ctre.WPI_VictorSPX(2)

        self.auto_right_motor = self.right_front_motor = ctre.WPI_TalonSRX(3)
        self.right_rear_motor = ctre.WPI_VictorSPX(4)


        self.grippers_left_motor = ctre.WPI_VictorSPX(6)
        self.grippers_right_motor = ctre.WPI_VictorSPX(9)

        self.left = wpilib.SpeedControllerGroup(
            self.left_front_motor, self.left_rear_motor
        )

        self.right = wpilib.SpeedControllerGroup(
            self.right_front_motor, self.right_rear_motor
        )

        self.drive = wpilib.drive.DifferentialDrive(
            self.left, self.right
        )

        self.hatch_panel_piston_solenoid = wpilib.DoubleSolenoid(1, 0)
        self.gripper_piston_solenoid = wpilib.DoubleSolenoid(4, 6)
        self.panel_mechanism_piston_solenoid = wpilib.DoubleSolenoid(2, 7)

        self.first_hatch_panel_piston_solenoid = wpilib.Solenoid(3)

        self.ramp_motor = ctre.WPI_TalonSRX(6)

        self.ramp_switch = wpilib.DigitalInput(0)

        self.ramp_cfg = MotorConfig(
            motor=self.ramp_motor,
            speed=1.0,
            forward_switch=None,
            backward_switch=self.ramp_switch
        )

        self.navx = navx.AHRS.create_i2c()
        self.navx.reset()

        self.right_joystick = wpilib.Joystick(0)
        self.left_joystick = wpilib.Joystick(1)

        self.controller = wpilib.XboxController(2)


    def testInit(self):
        self.auto_aligner.reset()

    def teleopInit(self):

        self.auto_aligner.reset()

    def teleopPeriodic(self):

        if self.controller.getStartButtonPressed():
            self.auto_aligner.enable()

        if self.controller.getBackButtonPressed():
            self.auto_aligner.reset()

        if self.controller.getBButtonPressed():
            self.navx.reset()
            self.gripper_piston.change_mode()

        if self.controller.getTriggerAxis(self.RIGHT_CONTROLLER_HAND) > 0.1:
            self.grippers.exhaust_ball()
        elif self.controller.getTriggerAxis(self.LEFT_CONTROLLER_HAND) > 0.1:
            self.grippers.intake_ball()

        if self.controller.getXButtonPressed():
            self.first_hatch_panel_piston.change_mode()

        if self.controller.getAButtonPressed():
            self.hatch_panel_piston.change_mode()



        if self.controller.getYButtonPressed():
            self.panel_mechanism_piston.change_mode()

        if self.controller.getBumper(self.RIGHT_CONTROLLER_HAND):
            self.ramp.set_forward()

        if self.controller.getBumper(self.LEFT_CONTROLLER_HAND):
            self.ramp.set_backward()

        if not self.drivetrain.locked:
            if self.right_joystick.getTrigger():
                self.drivetrain.tank_move(
                    -self.right_joystick.getY(),
                    -self.right_joystick.getY(),
                    True
                )
            else:
                self.drivetrain.tank_move(
                    -self.left_joystick.getY(),
                    -self.right_joystick.getY(),
                    True
                )

        self.tab.putNumber('Yaw', self.navx.getYaw())
        self.tab.putBoolean("Is vision enabled", self.auto_aligner.enabled)


if __name__ == "__main__":
    wpilib.run(MyRobot)
