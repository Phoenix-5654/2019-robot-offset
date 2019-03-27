import wpilib.buttons
from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from components.grippers import Grippers
from components.motor import Motor
from components.piston import Piston
from components.single_solenoid_piston import SingleSolenoidPiston


class TeleopMode(AutonomousStateMachine):
    drivetrain: Drivetrain
    grippers: Grippers
    hatch_panel_piston: Piston
    gripper_piston: Piston
    panel_mechanism_piston: Piston
    first_hatch_panel_piston: SingleSolenoidPiston
    ramp_pistons: SingleSolenoidPiston
    ramp: Motor
    controller: wpilib.XboxController
    right_joystick: wpilib.Joystick
    left_joystick: wpilib.Joystick

    RIGHT_CONTROLLER_HAND = wpilib.XboxController.Hand.kRight
    LEFT_CONTROLLER_HAND = wpilib.XboxController.Hand.kLeft

    MODE_NAME = 'Teleop Auto'
    DEFAULT = True

    def __init__(self):
        pass

    @state(first=True, must_finish=True)
    def teleop(self):

        if self.controller.getStartButtonPressed():
            self.drivetrain.lock()

        if self.controller.getBackButtonPressed():
            self.drivetrain.unlock()

        if self.controller.getStickButtonPressed(self.RIGHT_CONTROLLER_HAND):
            self.ramp_pistons.change_mode()

        if self.controller.getBButtonPressed():
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
