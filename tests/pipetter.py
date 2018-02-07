import ezstepper
import time

class Pipetter(object):
    def __init__(self):
        self.leftMotor = EZStepper(1, serialPort='/dev/ttyUSB0')
        self.rightMotor = EZStepper(1, serialPort='/dev/ttyUSB1')
        self.ejectDist = 5

    def setHome(self):
       leftmotor.setCurrentPosition(0)
       rightMotor.setCurrentPosition(0)

    def ejectFour(self):
        leftMotor.moveAbsolute(self.ejectDist)
        time.sleep(2)
        leftMotor.moveAbsolute(0)
        time.sleep(2)

    def ejectEight(self):
        leftMotor.moveAbsolute(self.ejectDist)
        time.sleep(2)
        rightMotor.moveAbsolute(self.ejectDist)
        time.sleep(2)
        rightMotor.moveAbsolute(0)
        time.sleep(2)
        leftMotor.moveAbsolute(0)
        time.sleep(2)
