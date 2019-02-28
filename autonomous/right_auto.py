import math

import ctre
import navx
import pathfinder as pf
from magicbot import AutonomousStateMachine, state

from autonomous.path_generator import PathGenerator
from components.drivetrain import Drivetrain


class RightAuto(AutonomousStateMachine):
    drivetrain: Drivetrain
    auto_left_motor: ctre.WPI_TalonSRX
    auto_right_motor: ctre.WPI_TalonSRX
    navx: navx.ahrs.AHRS

    MODE_NAME = "Right Auto"
    DEFAULT = True

    def setup(self):
        self.path_generator = PathGenerator(left_motor=self.auto_left_motor,
                                            right_motor=self.auto_right_motor,
                                            navx=self.navx)

    @state(first=True)
    def reset(self):
        self.auto_right_motor.setQuadraturePosition(0, 10)
        self.auto_left_motor.setQuadraturePosition(0, 10)
        self.navx.reset()

        self.next_state('execute_auto')

    @state(must_finish=True)
    def execute_auto(self):

        turn = self.path_generator.turn

        print(self.path_generator.left_output)
        print(self.path_generator.right_output)
        print(turn)

        self.drivetrain.tank_move(self.path_generator.left_output
                                  + turn,
                                  self.path_generator.right_output
                                  - turn)

        if self.path_generator.is_finished:
            self.done()
