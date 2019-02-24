import ctre
from magicbot import tunable

class Grippers:
    left_motor: ctre.VictorSPX
    right_motor: ctre.VictorSPX
    INTAKE_SPEED = tunable(default=0.5)
    EJECT_SPEED = tunable(default=0.5)

    def __init__(self):
        self.intake = self.exhaust = False

    def intake_ball(self):
        self.intake = True

    def exhaust_ball(self):
        self.exhaust = True

    def execute(self):
        if self.intake:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=self.INTAKE_SPEED)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=self.INTAKE_SPEED)
        elif self.exhaust:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=-self.EJECT_SPEED)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=-self.EJECT_SPEED)
        else:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=0)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=0)

        self.intake = self.exhaust = False
