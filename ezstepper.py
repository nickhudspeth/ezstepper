import serial
import time


class EZStepper(object):

    errorTable = {
        "addressError": ValueError("Cannot initialize EZStepper object without valid\
                    address."),
        "rangeError": ValueError("Commanded position out of range.")
    }

    def __init__(self, address=None, serialPort='/dev/ttyUSB0', baud=9600, timeout=1):
        if address is None:
            self.raiseError("rangeError")
        self.address = address
        self.ser = serial.Serial(
            port=serialPort, baudrate=baud, timeout=timeout, stopbits=1)
        self.setMaxMoveCurrent(100)  # Set max current to 100%  by default.
        self.linearMultiplier = 1
        self.microStepsPerStep = 256
        # self.setMicroStepResolution(self.microStepsPerStep)
        self.stepsPerRevolution = 200  # 1.8 degrees per step
        #self.mmPerRevolution = 0.100 * 25.4  # 0.1in pitch * 25.4 in/mm
        #self.mmPerRevolution = 1
        self.mmPerRevolution = 3.175
        self.linearMultiplier = int(round(self.stepsPerRevolution
            * self.microStepsPerStep
            / self.mmPerRevolution))
        self.currentPosition = 0
        self.currentVelocity = 1

    def raiseError(self, key):
        raise self.errorTable[key]

    def send(self, string):
        out = '/{}'.format(self.address) + string + 'R' + '\r\n'
        print(out)
        self.ser.write(out)

    def moveAbsolute(self, pos, units='mm'):
        """Microsteps or quadrature encoder ticks - 32-bit positioning."""
        if abs(pos) > pow(2, 31):
            self.raiseError("rangeError")
        pos = pos * self.linearMultiplier if units == 'mm' else pos
        self.send('A{}'.format(int(round(pos))))
        print ("Current Position = {}".format(self.currentPosition))
        print ("Pos = {}".format(pos))
        print ("Current Velocity = {}".format(self.currentVelocity))

        time = (pos-self.currentPosition)/(self.currentVelocity*self.linearMultiplier) if self.currentVelocity > 0 else "INF"
        print("Time = {}".format(time))
        self.setCurrentPosition(pos, "steps")

    def moveRelative(self, pos, units='mm'):
        """Move motor relative in negative direction (microsteps or quadrature
        encoder ticks.) (NOTE: for a finite move, the ending absolute position
        must be greater than zero.) A value of zero for the operand will cause
        an endless  move at speed V. (i.e. enter into Velocity Mode.)
        The velocity can then be changed on the fly by using the V command.
        An endless move can be terminated by issuing a T command or by a
        falling edge on the Switch2 input. NOTE: Ending position must be greater
        than zero."""
        if abs(pos) > pow(2, 31):
            self.raiseError("rangeError")
        if pos > 0:
            cmd = 'P'
            pos=abs(pos) * self.linearMultiplier if units == 'mm' else abs(pos)
            self.send('{}{}'.format(cmd, int(round(pos))))
        else:
            cmd = 'D'
            self.setCurrentPosition(abs(pos)+1)
            pos=abs(pos) * self.linearMultiplier if units == 'mm' else abs(pos)
            tmp = self.currentPosition / self.linearMultiplier
            self.send('{}{}'.format(cmd, int(round(pos))))
            self.setCurrentPosition(tmp - (pos/self.linearMultiplier))


    def home(self, maxSteps=400, units="mm"):
        """Home/initialize motor. Motor will turn toward 0 until the home
        opto sensor (opto #1) is interrupted. If already interrupted, it will
        back out of the opto and come back in until re-interrupted. Current
        motor position is set to zero."""
        if abs(maxSteps) > pow(2, 31):
            self.raiseError("rangeError")
        maxSteps=maxSteps *\
            self.linearMultiplier if units == 'mm' else maxSteps
        self.send('Z{}'.format(maxSteps))

    def setCurrentPosition(self, pos, units='mm'):
        """Change current position without moving. Sets current position
        to the position specified without moving the motor. New microstep
        position (preferably) should have the same remainder as the old
        position, when divided by 1024, else the motor may physically move/lose
        up to 2 steps when this command is issued. NOTE: This command must be
        issued after at least one A command, because the first A command
        initializes all registers to zero."""
        if abs(pos) > pow(2, 31):
            self.raiseError("rangeError")
        pos=pos * self.linearMultiplier if units == 'mm' else pos
        self.send('z{}'.format(pos))
        self.currentPosition = pos

    def setHomeFlagPolarity(self, polarity):
        """Sets polarity of home sensor, default value is 0."""
        pol=0 if polarity == 0 else 1
        self.send('f{}'.format(pol))

    def setInputPolarity(self, in1pol, in2pol, in3pol, in4pol):
        """Inverts each of the 4 main inputs depending on whether the binary
        interpretation of the 4-bit number has the bit zet or zero. This command
        is useful for toggling the polarity of the home flags and limits.
        E.G: /1ap7R will invert inputs 0, 1, and 2 (binary of 7 is 0111.)"""
        in1pol=1 if in1pol > 0 else 0
        in2pol=1 if in2pol > 0 else 0
        in3pol=1 if in3pol > 0 else 0
        in4pol=1 if in4pol > 0 else 0
        byte=0 | (in1pol << 3) | (in2pol << 2) | (in3pol << 1) | (in4pol)
        self.send('ap{}'.format(byte))

    def setPositiveRotationDirection(self, dir):
        """Change direction of rotation considered positive. This should only
        be done once on power-up. Do not use if in Encoder Feedback mode."""
        dir=1 if dir > 0 else 0
        self.send('F{}'.format(dir))

    def setVelocity(self, vel):
        """In position mode, sets max/slew speed of motor. Sets microsteps per
        second. It is recommended that his drive be left in 256 micro-step
        mode, since very high microsteps/sec numbers can be issues. If the
        encoder ratio (aE command) is set, the units of velocity change to
        encoder counts/second."""
        if vel == 'max':
            cmd = 16777216
        else:
            cmd = 16777216 if (abs(vel) > 16777216) else (abs(vel)*0.01*16777216)
        self.currentVelocity = cmd
        self.send('V{}'.format(int(cmd)))

    def setAccelerationFactor(self, factor):
        """In EZHR17EN, set acceleration factor. Accel in microsteps / sec^2 =
        (L Value)*(400,000,000/65536). E.g. using t = V/a /1L1R takes 16.384
        seconds to get to a speed of V=100000 microsteps/second. NOTE:
        Acceleration does not scale with encoder ratio."""
        factor=65000 if (abs(factor) > 65000) else abs(factor)
        self.send('L{}'.format(factor))

    def setMaxMoveCurrent(self, max):
        """For steppers "move" current on a scale of 0 to 100% of maximum
        current. 100% = 2A for EZHR17EN."""
        max=100 if (abs(max) > 100) else abs(max)
        self.maxMoveCurrent=max
        self.send('m{}'.format(max))

    def setMaxHoldCurrent(self, max):
        """Sets "Hold" current on a scale of 0 to 50% max current"""
        if max not in range(0, 50):
            self.raiseError("rangeError")
        self.maxHoldCurrent=max
        self.send("h{}".format(max))

    def setMicroStepResolution(self, res):
        """Adjusts the resolution in microsteps per step. It is recommended
        that step resolution be left at 256 microsteps (default.) It is
        recommended that his drive be left in 256 microstep mode. Only used
        reduced resolution if step and direction mode (n96) i selected and high
        "frequency step pulses cannot be generated. For best microstep results,
        a motor must be selected that is capable of microstep operation."""
        if res not in set([1, 2, 4, 8, 16, 32, 64, 128, 256]):
            self.raiseError("rangeError")
        self.send('j{}'.format(res))
        self.microStepsPerStep=res

    def setBaudRate(self, baud):
        """Adjusts baud rate. This command will usually be stored as program
        zero and execute on power-up Default baud rate is 9600. NOTE: correct
        termination and strict dais chaining required for reliable operation at
        higher baud rates."""
        if baud not in set([9600, 19200, 38400, 57600, 115200, 128000, 230400]):
            self.raiseError("rangeError")
        self.send('b{}'.format(baud))

    def delay(self, m):
        """Wait m number of milliseconds."""
        if m not in range(0, 30000):
            self.raiseError("rangeError")
        self.send('M{}'.format(m))

    def setBacklashCompensation(self, k):
        """Sets backlash compensation. When a nonzero value of k is specified,
        the drive will always approach the final position from a direction going
        mode negative. If going more positive, the drive will overshoot by an
        amount k and then go back. By always approaching from the same
        direction, the positioning will be more repeatable."""
        if k not in range(0, 65001):
            self.raiseError("rangeError")
        self.send('K{}'.format(k))

    def getCommandedMotorPosition(self):
        """Returns the current commanded motor position."""
        self.send('?0')
        return self.ser.readline()

    def getPositionModeMotorSpeed(self):
        """Returns the current slew/max speed for position mode."""
        self.send('?2')
        return self.ser.readline()

    def getInputStatus(self):
        """Returns the status of all four inputs, 0-15 representing a 4-bit
        binary pattern."""
        self.send('?4')
        return self.ser.readline()

    def getVelocityModeMotorSpeed(self):
        """Returns the current velocity mode speed."""
        self.send('?5')
        return self.ser.readline()

    def getMotorStepSize(self):
        """Returns the current step size microsteps/step/"""
        self.send('?6')
        return self.ser.readline()
