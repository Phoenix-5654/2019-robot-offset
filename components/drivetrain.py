import wpilib.drive
import navx
from networktables import NetworkTables


class Drivetrain:
    drive: wpilib.drive.DifferentialDrive
    navx: navx.ahrs.AHRS

    def __init__(self):
        self.navx_tab = NetworkTables.getTable('Navx')
        self.tank_enabled = self.locked = self.squared = False
        self.x = self.y = self.right = self.left = self.max_accel = self.max_velocity = 0

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

    def find_max_accel_and_velocity(self):

        curr_velocity = self.navx.getVelocityX()
        curr_accel = self.navx.getWorldLinearAccelX()

        if curr_velocity > self.max_velocity:
            self.max_velocity = curr_velocity

        if curr_accel > self.max_accel:
            self.max_accel = curr_accel

        self.navx_tab.putNumber('Max Velocity', self.max_velocity)
        self.navx_tab.putNumber('Max Accel', self.max_accel)

    def execute(self):
        if self.tank_enabled:
            self.drive.tankDrive(
                leftSpeed=self.left, rightSpeed=self.right, squareInputs=self.squared)
        else:
            self.drive.arcadeDrive(self.y, self.x, squareInputs=self.squared)

        self.find_max_accel_and_velocity()

        self.tank_enabled = False
