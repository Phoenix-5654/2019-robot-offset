import ctre


class Grippers:
    left_motor: ctre.VictorSPX
    right_motor: ctre.VictorSPX

    def __init__(self):
        self.intake = self.exhaust = False

    def intake_ball(self):
        self.intake = True

    def exhaust_ball(self):
        self.exhaust = True

    def execute(self):
        if self.intake:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=-0.7)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=0.7)
        elif self.exhaust:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=1)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=-1)
        else:
            self.left_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=0)
            self.right_motor.set(
                mode=ctre.ControlMode.PercentOutput, demand0=0)

        self.intake = self.exhaust = False
