from collections import namedtuple
MotorConfig = namedtuple(
    "MotorConfig", ['motor', 'speed', 'forward_switch', 'backward_switch'])


class Motor:
    cfg: MotorConfig

    def __init__(self):
        self.forward = self.backward = False

    def switch_pressed(self, switch):
        try:
            value = switch.get()
            print(value)
            return value
        except AttributeError:
            return False

    def set_forward(self):
        if not self.switch_pressed(self.cfg.forward_switch):
            self.forward = True

    def set_backward(self):
        if self.switch_pressed(self.cfg.backward_switch):
            self.backward = True

    def execute(self):
        if self.forward:
            self.cfg.motor.set(self.cfg.speed)
        elif self.backward:
            self.cfg.motor.set(-self.cfg.speed)
        else:
            self.cfg.motor.set(0)

        self.forward = self.backward = False
