import wpilib.drive


class Drivetrain:
    drive: wpilib.drive.DifferentialDrive

    def __init__(self):
        self.tank_enabled = self.locked = self.squared = False
        self.x = self.y = self.right = self.left = 0

    def move_x(self, x):
        self.x = x

    def move_y(self, y):
        self.y = y

    def move(self, x, y, squared=False):
        self.move_x(x)
        self.move_y(y)
        self.squared = squared

    def tank_move(self, left, right, squared=False):
        self.tank_enabled = True
        self.left = left
        self.right = right
        self.squared = squared

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    def execute(self):
        if self.tank_enabled:
            self.drive.tankDrive(
                leftSpeed=self.left, rightSpeed=self.right, squareInputs=self.squared)
        else:
            self.drive.arcadeDrive(self.y, self.x, squareInputs=self.squared)

        self.tank_enabled = False
