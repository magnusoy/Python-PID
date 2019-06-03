# #!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Importing time for time management
import time


class PID(object):
    """A class used to represent a PID - Controller

    ...

    Attributes
    ----------
    None

    Methods
    -------
    setIntegralLimits(lowerLimit, upperLimit)
        Limits the integral windup to the limits

    setOutputLimits(lowerLimit, upperLimit)
        Limits the output to the limits

    addOutputOffset(offset)
        Adds a offset to the computed output

    changeParameters(kp, ki, kd)
        Updates the controller terms

    compute(target, actual)
        Calculates the new output based on 
        the traditional PID formula
    """

    def __init__(self, kp, ki, kd, direction):
        """
        Parameters
        ----------
        kp : float
            Proportional gain
        ki : float
            Integral gain
        kd : float
            Derivative gain
        direction : int
            -1 for reverse
            1 for forward
        """

        self.direction = direction
        self.changeParameters(kp, ki, kd)
        self.updateTime = 100
        self.lastUpdate = self.millis(self)
        self.output = 0.0
        self.pOutput = 0.0
        self.iOutput = 0.0
        self.dOutput = 0.0
        self.lastActual = 0.0
        self.lowerIntegralLimit = 0
        self.upperIntegralLimit = 0

    def setIntegralLimits(self, lowerLimit, upperLimit):
        """Set the lower and upper limits for the intergral output

        Parameters
        ----------
        lowerlimit : float
            The lower limit
        upperlimit : float
            The upper limit
        """

        self.lowerIntegralLimit = lowerLimit
        self.upperIntegralLimit = upperLimit

    def setOutputLimits(self, lowerLimit, upperLimit):
        """Set the lower and upper limits for the total output

        Parameters
        ----------
        lowerlimit : float
            The lower limit
        upperlimit : float
            The upper limit
        """
        self.lowerOuputLimit = lowerLimit
        self.upperOutputLimit = upperLimit

    def addOutputOffset(self, offset):
        """Adds a offset to the output

        Parameters
        ----------
        offset : float
            Offset to be added to output
        """

        self.outputOffset = offset

    @staticmethod
    def millis(self):
        """Returns the current time in milliseconds

        Returns
        -------
        current time in milliseconds
        """

        return int(round(time.time() * 1000))

    def changeParameters(self, kp, ki, kd):
        """Update the controller terms

        Parameters
        ----------
        kp : float
            The proportional gain
        ki : float
            The integral gain
        kd : float
            The derivative gain
        """

        if self.direction < 0:
            self.kp = kp * -1
            self.ki = ki * -1
            self.kd = kd * -1
        else:
            self.kp = kp
            self.ki = ki
            self.kd = kd

    def compute(self, target, actual):
        """Calulates the output based on the PID algorithm

        Parameters
        ----------
        target : float
            Desired value
        actual : float
            Current value

        Returns
        -------
        output : float
            The output correction
        """

        now = self.millis(self)
        timeDifference = now - self.lastUpdate
        if timeDifference >= self.updateTime:
            error = target - actual
            self.pOutput = error * self.kp
            self.iOutput += error * self.ki
            self.dOutput += actual - self.lastActual

            if self.iOutput < self.lowerIntegralLimit:
                self.iOutput = self.lowerIntegralLimit
            elif self.iOutput > self.upperIntegralLimit:
                self.iOutput = self.upperIntegralLimit

            self.output = self.outputOffset + self.pOutput + self.iOutput + self.dOutput

            if self.output < self.lowerOuputLimit:
                self.output = self.lowerOuputLimit
            elif self.output > self.upperOutputLimit:
                self.output = self.upperOutputLimit

            self.lastActual = actual
            self.lastUpdate = now
        else:
            return self.output


# Example of usage
if __name__ == "__main__":
    pid = PID(kp=2.0, ki=0.0, kd=0.0, direction=1)
    pid.addOffset(50.0)
    pid.updateTime = 100
    pid.setOutputLimits(0, 100)

    while(True):
        output = pid.compute(50, 23)
        print(output)
