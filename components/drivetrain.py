import wpilib.drive


class Drivetrain:
    drive: wpilib.drive.DifferentialDrive

    def __init__(self):
        self.locked = False
        self.x = self.y = 0

    def move_x(self, x):
        self.x = x

    def move_y(self, y):
        self.y = y

    def move(self, x, y):
        self.move_x(x)
        self.move_y(y)

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    def execute(self):
        self.drive.arcadeDrive(self.y, self.x)
