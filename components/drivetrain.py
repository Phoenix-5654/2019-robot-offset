import wpilib.drive


class Drivetrain:
    drive: wpilib.drive.DifferentialDrive

    def __init__(self):
        self.tank_enabled = self.locked = False
        self.x = self.y = self.right = self.left = 0

    def move_x(self, x):
        self.x = x

    def move_y(self, y):
        self.y = y

    def move(self, x, y):
        self.move_x(x)
        self.move_y(y)

    def tank_move(self, left, right):
        self.tank_enabled = True
        self.left = left
        self.right = right

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    def execute(self):
        if self.tank_enabled:
            self.drive.tankDrive(leftSpeed=self.left, rightSpeed=self.right)
        else:
            self.drive.arcadeDrive(self.y, self.x)

        self.tank_enabled = False
