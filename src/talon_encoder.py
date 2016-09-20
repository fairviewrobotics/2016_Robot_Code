import wpilib
import math

class TalonEncoder(wpilib.Talon):

    def __init__(self, pinNumber, encoderAChannel, encoderBChannel, wheelDiameter):
        self(pinNumber)

        self.encoder = wpilib.Encoder(encoderAChannel, encoderBChannel)
        self.encoder.setDistancePerPulse(math.pi * wheelDiameter / 255)

        self.setDistance = 0

    def didReachSetDistance(self):
        return self.encoder.getDistance() == self.setDistance

    def setSpeed(self):
        speed = .5 if self.didReachSetDistance() else 0
        self.setSpeed(speed)

    def resetEncoder(self):
        self.encoder.reset()
