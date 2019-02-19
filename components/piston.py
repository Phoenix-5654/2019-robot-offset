import wpilib


class Piston:
    solenoid: wpilib.DoubleSolenoid

    CLOSED = wpilib.DoubleSolenoid.Value.kForward

    def __init__(self):
        self.value = self.CLOSED
        self.enable_change_mode = False

    def change_mode(self):
        self.enable_change_mode = True

    def execute(self):
        if self.enable_change_mode:
            self.value = 3 - self.value
            self.solenoid.set(wpilib.DoubleSolenoid.Value(self.value))

        self.enable_change_mode = False
