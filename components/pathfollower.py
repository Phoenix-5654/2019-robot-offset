import pathfinder as pf
import navx
import ctre
from components.drivetrain import Drivetrain


class PathFollower:

    navx: navx.ahrs.AHRS

    drivetrain: Drivetrain

    left_motor: ctre.WPI_TalonSRX
    right_motor: ctre.WPI_TalonSRX

    RESOLUTION = 1024
    WHEEL_DIAMETER = 0.1524
    MAX_ACCEL = 2.5
    MAX_VELOCITY = 2
    MAX_JERK = 60

    def __init__(self):

        points = []

        info, trajectory = pf.generate(
            points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
            dt=0.05,  # 50ms
            max_velocity=0,
            max_acceleration=0,
            max_jerk=self.MAX_JERK
        )

        modifier = pf.modifiers.TankModifier(trajectory).modify(0.5)

        self.left_follower = pf.followers.EncoderFollower(
            modifier.getLeftTrajectory())
        self.right_follower = pf.followers.EncoderFollower(
            modifier.getRightTrajectory())

        self.left_follower.configureEncoder(
            self.left_motor.getQuadraturePosition(), self.RESOLUTION, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(
            self.right_motor.getQuadraturePosition(), self.RESOLUTION, self.WHEEL_DIAMETER)

        self.left_follower.configurePIDVA(
            1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)
        self.right_follower.configurePIDVA(
            1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)

        self.enabled = False

    def enable(self):
        self.enabled = True

    def execute(self):
        if self.enabled:
            left_output = self.left_follower.calculate(
                self.left_motor.getQuadraturePosition())
            right_output = self.right_follower.calculate(
                self.right_motor.getQuadraturePosition())

            gyro_heading = self.navx.getYaw()
            desired_heading = pf.r2d(self.left_follower.getHeading())

            angle_difference = pf.boundHalfDegrees(
                desired_heading - gyro_heading)

            turn = 0.8 * (-1.0 / 80.0) * angle_difference

            self.drivetrain.tank_move(left_output + turn, right_output - turn)

            if self.left_follower.isFinished() and self.right_follower.isFinished():
                self.enabled = False
