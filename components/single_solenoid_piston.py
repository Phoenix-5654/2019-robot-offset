import wpilib


class SingleSolenoidPiston:
    solenoid: wpilib.Solenoid
    CLOSED = False

    def __init__(self):
        self.value = self.CLOSED
        self.enable_change_mode = False

    def change_mode(self):
        self.enable_change_mode = True

    def execute(self):
        if self.enable_change_mode:
            if self.value == True:
                self.value = False
            else:
                self.value = True

            self.solenoid.set(self.value)

        self.enable_change_mode = False
