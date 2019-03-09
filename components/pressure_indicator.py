import wpilib
from networktables import NetworkTables


class PressureIndicator:
    sensor: wpilib.AnalogInput

    def __init__(self):
        self.tab = NetworkTables.getTable('Pneumatics')

    def execute(self):
        self.tab.putNumber('Voltage', self.sensor.getVoltage())
        self.tab.putNumber('Pressure', 250.0 *
                           (self.sensor.getVoltage() / 5.0) - 25.0)
