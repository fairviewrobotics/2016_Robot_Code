class ShootCommand:
    autoIsShooting = False
    autoHasShot = False
    done = False
    bottomShooterSetSpeed = -.6
    topShooterSetSpeed = 0.7
    bottomEncoderRateToSpeed = .000015
    topEncoderRateToSpeed = .000023

    def __init__(self, myRobot, bottomShooter, topShooter, bottomShooterEncoder, topShooterEncoder, solenoid):
        self.myRobot = myRobot
        self.autoHasShotTimer = wpilib.Timer()
        self.bottomShooter = bottomShooter
        self.topShooter = topShooter
        self.bottomShooterEncoder = bottomShooterEncoder
        self.topShooterEncoder = topShooterEncoder
        self.rampSolenoid = solenoid

    def update(self):
        if not self.autoIsShooting:
            self.autoIsShooting = True
            self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            self.bottomShooter.setSpeed(self.bottomShooterSetSpeed)
            self.topShooter.setSpeed(self.topShooterSetSpeed)

        if -self.bottomShooterEncoder.getRate()*  self.bottomEncoderRateToSpeed >= self.bottomShooterSetSpeed and -self.topShooterEncoder.getRate() * self.topEncoderRateToSpeed >= self.topShooterSetSpeed:
            print("Shooting")
            self.autoIsShooting = False
            self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            self.autoHasShotTimer.reset()
            self.autoHasShotTimer.start()
            self.hasShot = True

        if self.autoHasShot:
            if self.autoHasShotTimer.get() > 1:
                self.bottomShooter.setSpeed(0.0)
                self.topShooter.setSpeed(0.0)
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.autoHasShot = False
                self.done = True
    def checkIfDone(self):
        if self.done:
            return True

class Command:
    UnitMultiplier = 3000
    tolerance = 50
    radius = 10.125
    speed = .6
    decayCof = 1
    approachCof = .07
    alignApproachCof = .04
    alignDecayCof = 1

    def __init__(self, angle, dist, myRobot, leftEncoder, rightEncoder, action="drive"):

        print("Creating command object")
        self.dist = dist
        self.angle = angle

        self.myRobot = myRobot

        self.action = action

        self.leftEncoder = leftEncoder
        self.rightEncoder = rightEncoder

        if angle == 0.0:
            self.leftDistance = dist
            self.rightDistance = -dist
        else:
            self.leftDistance = math.pi * 2.0 * self.radius * (self.angle / 360.0)
            self.rightDistance = math.pi * 2.0 * self.radius * (self.angle / 360.0)

    def update(self):
        #print("Right Distance Left "+str(abs(self.rightDistance - self.rightEncoder.getDistance())))
        #print("Left Distance Left "+str(abs(self.leftDistance - self.leftEncoder.getDistance())))
        rightSpeed = 0.0
        leftSpeed = 0.0

        if self.rightDistance > self.rightEncoder.getDistance():
            rightSpeed = -(self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.rightDistance - self.rightEncoder.getDistance())))))))
        elif self.rightDistance < self.rightEncoder.getDistance():
            rightSpeed = (self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.rightDistance - self.rightEncoder.getDistance())))))))

        if self.leftDistance > self.leftEncoder.getDistance():
            leftSpeed = (self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.leftDistance - self.leftEncoder.getDistance())))))))
        elif self.leftDistance < self.leftEncoder.getDistance():
            leftSpeed = -(self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.leftDistance - self.leftEncoder.getDistance())))))))

        if abs(self.rightDistance - self.rightEncoder.getDistance()) > abs(self.leftDistance - self.leftEncoder.getDistance()):
            rightSpeed = rightSpeed*(2.0/(1+(self.alignDecayCof*math.e**(-abs(self.alignApproachCof*(abs(self.leftDistance - self.leftEncoder.getDistance()) - abs(self.rightDistance - self.rightEncoder.getDistance())))))))
        elif abs(self.leftDistance - self.leftEncoder.getDistance()) > abs(self.rightDistance - self.rightEncoder.getDistance()):
            leftSpeed = leftSpeed*(2.0/(1+(self.alignDecayCof*math.e**(-abs(self.alignApproachCof*(abs(self.leftDistance - self.leftEncoder.getDistance()) - abs(self.rightDistance - self.rightEncoder.getDistance())))))))

        self.myRobot.tankDrive(leftSpeed, rightSpeed)
        
    def checkIfDone(self):
        return abs(self.leftEncoder.getDistance() - self.leftDistance) <= self.tolerance and abs(self.rightEncoder.getDistance() - self.rightDistance) <= self.tolerance
