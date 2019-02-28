import os.path
import pickle

import ctre
import navx
import pathfinder as pf
import wpilib


class PathGenerator:
    RESOLUTION = 1024
    WHEEL_DIAMETER = 0.1524
    MAX_ACCEL = 2
    MAX_VELOCITY = 1.5
    MAX_JERK = 60
    CYCLE_TIME = 0.02  # 20ms
    TIMEOUT_MS = 10
    WHEELBASE_WIDTH = 0.69

    # left_motor: ctre.WPI_TalonSRX
    # right_motor: ctre.WPI_TalonSRX
    # navx: navx.ahrs.AHRS

    def __init__(self, points: list,
                 left_motor: ctre.WPI_TalonSRX,
                 right_motor: ctre.WPI_TalonSRX,
                 navx: navx.ahrs.AHRS,
                 file_name="trajectory.pickle"):

        self.right_motor = right_motor
        self.left_motor = left_motor
        self.navx = navx

        self.right_motor.setQuadraturePosition(0, self.TIMEOUT_MS)
        self.left_motor.setQuadraturePosition(0, self.TIMEOUT_MS)
        self.navx.reset()

        pickle_file = os.path.join(os.path.dirname(__file__), "../PathWeaver/output/right_auto.")
        # if wpilib.RobotBase.isSimulation():
        #     info, trajectory = pf.generate(
        #         points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
        #         dt=self.CYCLE_TIME,
        #         max_velocity=self.MAX_VELOCITY,
        #         max_acceleration=self.MAX_ACCEL,
        #         max_jerk=self.MAX_JERK
        #     )
        #     with open(pickle_file, 'wb') as fp:
        #         pickle.dump(trajectory, fp)
        # else:
        #     with open(pickle_file, 'rb') as fp:
        #         trajectory = pickle.load(fp)
        left_trajectory = pf.deserialize_csv(pickle_file + "left.pf1.csv")
        right_trajectory = pf.deserialize_csv(pickle_file+ "right.pf1.csv")

        self.left_follower = pf.followers.EncoderFollower(
            left_trajectory)
        self.right_follower = pf.followers.EncoderFollower(
            right_trajectory)

        self.left_follower.configureEncoder(
            self.left_motor.getQuadraturePosition(),
            self.RESOLUTION, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(
            self.left_motor.getQuadraturePosition(),
            self.RESOLUTION, self.WHEEL_DIAMETER)

        self.left_follower.configurePIDVA(
            1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)
        self.right_follower.configurePIDVA(
            1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)

    @property
    def left_output(self):
        return self.left_follower.calculate(
            self.left_motor.getQuadraturePosition())

    @property
    def right_output(self):
        return self.right_follower.calculate(
            self.right_motor.getQuadraturePosition())

    @property
    def turn(self):
        return 0
        gyro_heading = self.navx.getYaw()
        desired_heading = pf.r2d(self.left_follower.getHeading())

        angle_difference = pf.boundHalfDegrees(
            desired_heading - gyro_heading)

        return 0.8 * (-1.0 / 80.0) * angle_difference

    @property
    def is_finished(self):
        return self.left_follower.isFinished() and self.right_follower.isFinished()
