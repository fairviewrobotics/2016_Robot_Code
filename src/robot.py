#!/usr/bin/env python3

import wpilib
import time
from networktables import NetworkTable
import math
import queue

import logging

logging.basicConfig(level=logging.DEBUG)

# ( ﾉ ﾟｰﾟ)ﾉ☀ PRAISE THE SUN ☀ ヽ(ﾟｰﾟヽ)
class ShootCommand:
    autoIsShooting = False
    autoHasShot = False
    autoShootingUp = False
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
            self.autoShootingUp = True

        if self.autoShootingUp and -self.bottomShooterEncoder.getRate() * self.bottomEncoderRateToSpeed >= self.bottomShooterSetSpeed and -self.topShooterEncoder.getRate() * self.topEncoderRateToSpeed >= self.topShooterSetSpeed:
            print("SHOOTING")
            self.autoHasShot = True
            self.autoShootingUp = False
            self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            self.autoHasShotTimer.reset()
            self.autoHasShotTimer.start()

        if self.autoHasShot:
            if self.autoHasShotTimer.get() > .5:
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
    decayCof = 2
    approachCof = .07
    alignApproachCof = .04
    alignDecayCof = 1

    def __init__(self, angle, dist, leftWheelsPID, rightWheelsPID, rampSolenoid, action="drive"):

        print("Creating Command Object")
        self.dist = dist
        self.angle = angle

        self.rampSolenoid = rampSolenoid
        self.action = action

        self.leftWheelsPID = leftWheelsPID
        self.rightWheelsPID = rightWheelsPID

        if angle == 0.0:
            self.leftDistance = dist
            self.rightDistance = -dist
        else:
            self.leftDistance = -math.pi * 2.0 * self.radius * (self.angle / 360.0)
            self.rightDistance = math.pi * 2.0 * self.radius * (self.angle / 360.0)

    # def update(self):
    #     print("Right Distance Left "+str(abs(self.rightDistance - self.rightEncoder.getDistance())))
    #     print("Left Distance Left "+str(abs(self.leftDistance - self.leftEncoder.getDistance())))
    #     rightSpeed = 0.0
    #     leftSpeed = 0.0
    #
    #     self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
    #     if self.rightDistance > self.rightEncoder.getDistance():
    #         rightSpeed = -(self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.rightDistance - self.rightEncoder.getDistance())))))))
    #     elif self.rightDistance < self.rightEncoder.getDistance():
    #         rightSpeed = (self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.rightDistance - self.rightEncoder.getDistance())))))))
    #
    #     if self.leftDistance > self.leftEncoder.getDistance():
    #         leftSpeed = (self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.leftDistance - self.leftEncoder.getDistance())))))))
    #     elif self.leftDistance < self.leftEncoder.getDistance():
    #         leftSpeed = -(self.speed/(1+(self.decayCof*(math.e**(-abs(self.approachCof*(self.leftDistance - self.leftEncoder.getDistance())))))))
    #
    #     if abs(self.rightDistance - self.rightEncoder.getDistance()) > abs(self.leftDistance - self.leftEncoder.getDistance()):
    #         rightSpeed = rightSpeed*(2.0/(1+(self.alignDecayCof*math.e**(-abs(self.alignApproachCof*(abs(self.leftDistance - self.leftEncoder.getDistance()) - abs(self.rightDistance - self.rightEncoder.getDistance())))))))
    #     elif abs(self.leftDistance - self.leftEncoder.getDistance()) > abs(self.rightDistance - self.rightEncoder.getDistance()):
    #         leftSpeed = leftSpeed*(2.0/(1+(self.alignDecayCof*math.e**(-abs(self.alignApproachCof*(abs(self.leftDistance - self.leftEncoder.getDistance()) - abs(self.rightDistance - self.rightEncoder.getDistance())))))))
    #
    #     self.myRobot.tankDrive(leftSpeed, rightSpeed)
    def update(self):
        self.leftWheelsPID.setSetpoint(self.leftDistance)
        self.rightWheelsPID.setSetpoint(self.rightDistance)

        #print("Left Wheel Setpoint: " + str(self.leftWheelsPID.getSetpoint()) )
        #print("Right Wheel Setpoint: " + str(self.rightWheelsPID.getSetpoint()) )

    def checkIfDone(self):
        return self.rightWheelsPID.onTarget() and self.leftWheelsPID.onTarget()
class MyRobot(wpilib.IterativeRobot):

    radius = 10.125

    joystick = 0
    flightstick = 2

    # Channel Definitions
    leftWheelsChannel = 0
    rightWheelsChannel = 1

    bottomShooterChannel = 2
    topShooterChannel = 3

    solenoidShiftReverse = 5
    solenoidShiftForward = 6

    # Pickup Mechanism
    intakeMotorSetSpeed = -.9
    pickupWaitTime = 3

    # Shooting Mechanism
    bottomShooterSetSpeed = -0.59
    topShooterSetSpeed = 0.66

    bottomEncoderRateToSpeed = .000015
    topEncoderRateToSpeed = .000023

    bottomShooterReverseSetSpeed = 0.4
    topShooterReverseSetSpeed = -0.4

    # Dank Lights
    lightArr = [[False, False, False], [True, False, False], [False, True, False], [False, False, True], [True, True, False], [False, True, True], [True, True, True]]

    # Autonomous commands
    commands = None
    currentCommand = None
    autoCommandCooldown = False
    autoCommandCooldownTime = 0.5

    # Shooting booleans
    readyForShoot = False
    isShooting = False
    hasShot = False
    shootManualOveride = False
    isShootClearing = False
    reverseShootermanualOverride = False

    intakeManualOverride = False
    pickUpRequested = False
    pickUpRequestedAllowed = False

    # Ramp booleans
    readyForRampToggle = False
    rampUp = False

    # Button Booleans
    shootRequested = False
    shootRequestedAllow = False
    flightStickShootState = False

    flightStickRampState = False

    # Buttons
    joyStickShoot = 1
    #joyStickPickUp = 2
    joyStickAutoShoot = 4
    joyStickRampUp = 6
    joyStickRampDown = 5
    joyStickManualIntake = 3
    joyStickShiftUp = 8
    joyStickShiftDown = 7
    joyStickReverseShooter = 2

    flightStickShoot = 10
    flightStickReverseShooter = 10

    flightStickIntakeAxis = 0
    flightSticksideAxis = 4
    flightStickForwardAxis = 1
    flightStickShiftAxis = 2

    isAutoShoot = False;
    toAngle = 0;
    toDistance = 0;
    isAngleTurn = False;
    PIDSet = False;
    autoMove = False;
    autoShooting = False
    def robotInit(self):
        #NetworkTable
        #NetworkTable.setIPAddress('roborio-2036-frc.local')
        #NetworkTable.setClientMode()
        #NetworkTable.initialize()
        self.table = NetworkTable.getTable("goalvals")
        def q():
              print(table.getNumber("distance"),table.getNumber("angle"))


        # Joysticks 1 & 2 on the driver station
        self.joyStick = wpilib.Joystick(self.joystick)
        self.flightStick = wpilib.Joystick(self.flightstick)

        # Drivetrain motors
        self.leftWheelsMotor = wpilib.Talon(0)
        self.rightWheelsMotor = wpilib.Talon(1)
        self.leftWheelsMotor.setInverted(True)
        self.rightWheelsMotor.setInverted(True)

        # Object that handles basic drive operations
        self.myRobot = wpilib.RobotDrive(self.leftWheelsMotor, self.rightWheelsMotor)
        self.myRobot.setExpiration(0.1)
        self.myRobot.setSafetyEnabled(True)

        # Encoder setups
        self.leftEncoder = wpilib.Encoder(0, 1)
        self.rightEncoder = wpilib.Encoder(2, 3)
        self.leftEncoder.setDistancePerPulse(.027612)# This is totaly the wrong number
        self.rightEncoder.setDistancePerPulse(.027612)

        # # Autonomous
        # self.currCommand = None
        # self.autoInProgress = True
        self.autoTimer = wpilib.Timer()
        self.autoRampTimer = wpilib.Timer()
        self.hasShotTimer = wpilib.Timer()
        self.pickUpTimer = wpilib.Timer()
        self.shootClearTimer = wpilib.Timer()
        self.flightStickRampTimer = wpilib.Timer()

        # PID Controllers for the Drivetrain Kp, Ki, Kd, PIDSource, PIDOutput
        self.leftWheelsPID = wpilib.PIDController(.01, 0.0, 0.0, self.leftEncoder, self.leftWheelsMotor)
        self.rightWheelsPID = wpilib.PIDController(.01, 0.0, 0.0, self.rightEncoder, self.rightWheelsMotor)
        self.rightWheelsPID.setPercentTolerance(5)
        self.leftWheelsPID.setPercentTolerance(5)

        # Shooter mechanism
        self.bottomShooter = wpilib.Talon(self.bottomShooterChannel)
        self.topShooter = wpilib.Talon(self.topShooterChannel)

        self.bottomShooterEncoder = wpilib.Encoder(6, 7)
        self.topShooterEncoder = wpilib.Encoder(8, 9)

        # Motor for intake mechanism
        self.intake = wpilib.Talon(9)

        # Compresser
        self.compressor = wpilib.Compressor()
        self.compressor.start()

        # Solenoid for shifting
        self.shiftSolenoid = wpilib.DoubleSolenoid(2, 3)
        self.lightSolenoid = wpilib.Solenoid(4)
        # Solenoids for RGB Values for Lights
        self.zeroLightSolenoid = wpilib.Solenoid(5)
        self.oneLightSolenoid = wpilib.Solenoid(6)
        self.twoLightSolenoid = wpilib.Solenoid(7)

        # Timer for lights to avoid advincing multiple indexes in lightArr with one button press.
        self.lightTimer = wpilib.Timer()
        self.lightTimer.start()

        # Index in lightArr to signify what rgb values are turned on
        self.lightIndex = 0

        # Solenoid to lift ball to the shoother meachanism
        self.rampSolenoid = wpilib.DoubleSolenoid(0, 1)

    def autonomousInit(self):
        print("AutoInit")
        self.autoTimer.reset()
        self.autoTimer.start()
        # #self.release()
        # self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        #
        # #Command queue initialization
        # self.commands = queue.Queue()
        # # self.commands.put(ShootCommand(self.myRobot, self.bottomShooter,self.topShooter,self.bottomShooterEncoder, self.topShooterEncoder, self.rampSolenoid))
        # self.commands.put(Command(0.0, 10000.0, self.leftWheelsPID, self.rightWheelsPID, self.rampSolenoid))
        # # self.commands.put(Command(0.0, -100.0, self.myRobot, self.rightEncoder, self.leftEncoder, self.rampSolenoid))
        # # self.commands.put(Command(0.0, 100.0, self.myRobot, self.rightEncoder, self.leftEncoder, self.rampSolenoid))
        # # self.commands.put(Command(0.0, -100.0, self.myRobot, self.rightEncoder, self.leftEncoder, self.rampSolenoid))
        # # self.commands.put(Command(90, 0.0, self.myRobot, self.rightEncoder, self.leftEncoder, self.rampSolenoid))
        #
        # self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        # self.commands.put(Command(0.0, 0.0, self.myRobot, self.rightEncoder, self.leftEncoder, self.rampSolenoid))
        # self.currCommand = self.commands.get()
        #
        # self.autoRampTimer.reset()
        # self.autoRampTimer.start()
        # self.autoInProgress = True

    def autonomousPeriodic(self):
        if self.autoTimer.get() < 4:
            self.leftWheelsMotor.set(-.9)
            self.rightWheelsMotor.set(.9)
        else:
            self.leftWheelsMotor.set(0)
            self.rightWheelsMotor.set(0)
        # if self..get() < .1:
        #     self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        # else:
        #     #print("Lifting Ramp")
        #     self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        #
        # if self.commands.qsize() > 0:
        #     if self.currCommand == None:
        #         pass
        #             #print("Staring Command Queue")
        #             #self.currCommand = self.commands.get()
        #             #self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        #     else:
        #         if self.currCommand.checkIfDone() and not self.autoCommandCooldown:
        #             print("Initiating next Command")
        #             self.autoCommandCooldown = True
        #             self.autoTimer.start()
        #         else:
        #             #print("Updating!")
        #             self.currCommand.update()
        #
        #     if self.autoCommandCooldown:
        #         print("Cooldown")
        #         if self.autoTimer.get() > self.autoCommandCooldownTime:
        #             print("Cooldown Over")
        #             self.rightEncoder.reset()
        #             self.leftEncoder.reset()
        #             self.leftWheelsPID.reset()
        #             self.rightWheelsPID.reset()
        #             self.currCommand = self.commands.get()
        #             self.autoCommandCooldown = False
        #             self.autoTimer.stop()
        #             self.autoTimer.reset()
        #
        # else:
        #     print("End of queue")
        #     self.myRobot.tankDrive(0.0, 0.0)
        #     self.topShooter.setSpeed(0.0)
        #     self.bottomShooter.setSpeed(0.0)
        #     self.intake.setSpeed(0.0)
                    #print("Waiting For Auto Cooldown")
        wpilib.Timer.delay(0.005)
    def teleopInit(self):
        self.updateLights()
        self.readyForShoot = True
        self.shootRequestedAllow = True
        self.pickUpRequestedAllowed = True
        self.readyForPickUp = True
        self.rampUp = True
        self.rampToggleRequestedAllowed = True

        self.rightEncoder.reset()
        self.leftEncoder.reset()
        self.topShooterEncoder.reset()
        self.bottomShooterEncoder.reset()
        self.reverseShootermanualOverride = False
        self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        self.rampUp = True

        self.leftWheelsPID.disable()
        self.rightWheelsPID.disable()

    def teleopPeriodic(self):
        try:
            print("Dist:")
            print(str(self.table.getNumber("distance")))
            print("Angle:")
            print(str(self.table.getNumber("angle")))
        except KeyError:
            print("key error")

        self.lightSolenoid.set(True)
        if self.joyStick.getRawButton(self.joyStickAutoShoot) and not self.isAutoShoot:
            print("AUTO SHOOTING INITIATED")
            try:
                self.toAngle = 354#self.table.getNumber("angle")##GET ANGLE
                self.toDistance = 56#self.table.getNumber("distance")#getDistance
                self.isAutoShoot = True
                self.isAngleTurn = True
                self.autoMove = False
                self.PIDSet = False
                self.autoShooting = False
                self.leftWheelsPID.enable()
                self.rightWheelsPID.enable()
                self.leftWheelsPID.reset()
                self.rightWheelsPID.reset()
            except KeyError:
                pass

        elif self.isAutoShoot:
            if self.isAngleTurn and not self.PIDSet:
                print("AUTOSHOOT SETTING ANGLE TURN")
                self.leftWheelsPID.reset()
                self.rightWheelsPID.reset()
                self.leftWheelsPID.setSetpoint(self.toAngle * 2 * 3.14159 *self.radius)
                self.rightWheelsPID.setSetpoint(self.toAngle * 2 * 3.14159 *self.radius)
                self.PIDSet = True
            elif self.isAngleTurn and self.PIDSet:
                if self.rightWheelsPID.onTarget() and self.leftWheelsPID.onTarget():
                    print("AUTOSHOOT ANGLE TURN FINISHED")
                    self.PIDSet = False
                    self.isAngleTurn = False
                    self.autoMove = True
            elif self.autoMove and not self.PIDSet:
                print("AUTOSHOOT SETTING MOVE TO")
                self.leftWheelsPID.reset()
                self.rightWheelsPID.reset()
                self.leftWheelsPID.setSetpoint(self.toDistance)
                self.rightWheelsPID.setSetpoint(self.toDistance)
                self.PIDSet = True
            elif self.autoMove and self.PIDSet:
                if self.rightWheelsPID.onTarget() and self.leftWheelsPID.onTarget():
                    print("AUTOSHOOT MOVE FINISHED")
                    self.PIDSet = False
                    self.autoMove = False
                    self.autoShooting = True
                    self.readyForShoot = True
            elif self.autoShooting:
                print("AUTOSHOOT SHOOTING BALL")
                self.leftWheelsPID.disable()
                self.rightWheelsPID.disable()
                #Shooting Algorithm Here
                if self.readyForShoot:
                    print("Shoot requested")
                    self.readyForShoot = False
                    self.isShooting = False
                    self.isShootClearing = True
                    self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                    self.rampUp = True
                    self.bottomShooter.setSpeed(self.bottomShooterReverseSetSpeed)
                    self.topShooter.setSpeed(self.topShooterReverseSetSpeed)
                    self.intake.setSpeed(self.intakeMotorSetSpeed)
                    self.shootClearTimer.reset()
                    self.shootClearTimer.start()

                if self.isShootClearing:
                    if self.shootClearTimer.get() > .45:
                        print("Finished Clearing")
                        print(str(self.isShootClearing))
                        self.isShootClearing = False
                        self.isShooting = True
                        self.bottomShooter.setSpeed(self.bottomShooterSetSpeed)
                        self.topShooter.setSpeed(self.topShooterSetSpeed)

                if self.isShooting and not self.isShootClearing:
                    if -self.bottomShooterEncoder.getRate() * self.bottomEncoderRateToSpeed >= self.bottomShooterSetSpeed and -self.topShooterEncoder.getRate() * self.topEncoderRateToSpeed >= self.topShooterSetSpeed:
                        self.isShooting = False
                        print((self.isShooting))
                        print("Shooting")
                        self.hasShot = True
                        self.isShooting = False
                        self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                        self.rampUp = False
                        self.hasShotTimer.reset()
                        self.hasShotTimer.start()

                if self.hasShot:
                    if self.hasShotTimer.get() > .25:
                        print("Shoot cycle finished")
                        self.intake.setSpeed(0.0)
                        # i love guysB
                        self.bottomShooter.setSpeed(0.0)
                        self.topShooter.setSpeed(0.0)
                        self.hasShot = False
                        self.autoShooting = False
                #End of shoot

            else:
                print("AUTOSHOOT SEQUENCE FINISHED")
                self.isAutoShoot = False
                self.leftWheelsPID.disable()
                self.rightWheelsPID.disable()

        else:
            # Manual intake control
            if self.joyStick.getRawButton(self.joyStickManualIntake):
                print("Manual intake")
                self.intakeManualOverride = True
                self.intake.setSpeed(self.intakeMotorSetSpeed)
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                if not self.isShooting and not self.hasShot:
                    self.topShooter.setSpeed(.3)
                print(str(self.intakeMotorSetSpeed))

            elif self.intakeManualOverride and not self.isShootClearing and not self.isShooting:
                print("Manual override over")
                self.intake.setSpeed(0.0)
                self.intakeManualOverride = False

            leftWheel = 2*self.flightStick.getRawAxis(self.flightStickForwardAxis)
            rightWheel = 2*self.flightStick.getRawAxis(self.flightStickForwardAxis)
            leftWheel -= 2*self.flightStick.getRawAxis(self.flightSticksideAxis)
            rightWheel += 2*self.flightStick.getRawAxis(self.flightSticksideAxis)

            if leftWheel > 1:
                leftWheel = 1
            elif leftWheel < -1:
                leftWheel = -1
            if rightWheel > 1:
                rightWheel = 1
            elif rightWheel < -1:
                rightWheel = -1

            if 2*self.flightStick.getRawAxis(self.flightStickForwardAxis) < -1:
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                self.rampUp = True

            # Going forward
            if leftWheel == rightWheel:
                print("Right Encoder: " + str(self.rightEncoder.getDistance()) + ", Left Encoder: " + str(self.leftEncoder.getDistance()))
            else:
                self.rightEncoder.reset()
                self.leftEncoder.reset()

            self.myRobot.tankDrive(leftWheel, rightWheel)


            if not self.intakeManualOverride:
                speed = 2.0 * self.flightStick.getRawAxis(self.flightStickIntakeAxis)
                #print(str(speed) +" "+ str(2*self.flightStick.getRawAxis(self.flightStickForwardAxis)))
                if speed < -1.0 and not 2 * self.flightStick.getRawAxis(self.flightStickForwardAxis) < -1:
                    self.rampUp = True
                    self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                if abs(speed) > .3:
                    pass
                    self.intake.setSpeed(speed)
                else:
                    pass
                    self.intake.setSpeed(0.0)

            # Button Checks
            if self.joyStick.getRawButton(self.joyStickShoot): #self.flightStick.getRawButton(self.flightStickShoot):
                if self.shootRequestedAllow:
                    self.shootRequested = True
                    self.shootRequestedAllow = False
            else:
                self.shootRequestedAllow = True

            # Shooting
            if self.readyForShoot:
                if self.shootRequested:
                    print("Shoot requested")
                    self.readyForShoot = False
                    self.isShooting = False
                    self.isShootClearing = True
                    self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                    self.rampUp = True
                    self.bottomShooter.setSpeed(self.bottomShooterReverseSetSpeed)
                    self.topShooter.setSpeed(self.topShooterReverseSetSpeed)
                    self.intake.setSpeed(self.intakeMotorSetSpeed)
                    self.shootRequested = False
                    self.shootClearTimer.reset()
                    self.shootClearTimer.start()

            if self.isShootClearing:
                if self.shootClearTimer.get() > .45:
                    print("Finished Clearing")
                    print(str(self.isShootClearing))
                    self.isShootClearing = False
                    self.isShooting = True
                    self.bottomShooter.setSpeed(self.bottomShooterSetSpeed)
                    self.topShooter.setSpeed(self.topShooterSetSpeed)

            if self.isShooting and not self.isShootClearing:
                if -self.bottomShooterEncoder.getRate() * self.bottomEncoderRateToSpeed >= self.bottomShooterSetSpeed and -self.topShooterEncoder.getRate() * self.topEncoderRateToSpeed >= self.topShooterSetSpeed:
                    self.isShooting = False
                    print((self.isShooting))
                    print("Shooting")
                    self.hasShot = True
                    self.isShooting = False
                    self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                    self.rampUp = False
                    self.hasShotTimer.reset()
                    self.hasShotTimer.start()

            if self.hasShot:
                if self.hasShotTimer.get() > .25:
                    print("Shoot cycle finished")
                    self.intake.setSpeed(0.0)
                    # i love guysB
                    self.bottomShooter.setSpeed(0.0)
                    self.topShooter.setSpeed(0.0)
                    self.hasShot = False
                    self.readyForShoot = True

                if not self.isShooting and not self.hasShot and not self.reverseShootermanualOverride and not self.isShootClearing:
                    self.topShooter.setSpeed(-.5*self.flightStick.getRawAxis(self.flightStickIntakeAxis))
            #RampControl

            if self.joyStick.getRawButton(self.joyStickRampUp):
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                self.rampUp = True
                print("Ramp up")

            if self.joyStick.getRawButton(self.joyStickRampDown):
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.rampUp = False
                print("Ramp down")

            if self.joyStick.getRawButton(self.joyStickReverseShooter) and not self.reverseShootermanualOverride:
                self.topShooter.setSpeed(self.topShooterReverseSetSpeed)
                self.bottomShooter.setSpeed(self.bottomShooterReverseSetSpeed)
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                self.rampUp = False
                self.reverseShootermanualOverride = True
            elif self.reverseShootermanualOverride and not self.joyStick.getRawButton(self.joyStickReverseShooter):
                self.reverseShootermanualOverride = False
                self.rampUp = True
                self.rampSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                self.topShooter.setSpeed(0.0)
                self.bottomShooter.setSpeed(0.0)
            #ShiftingGears
            if self.flightStick.getRawAxis(self.flightStickShiftAxis) > 0.5 or self.joyStick.getRawButton(self.joyStickShiftUp):
                self.shiftSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                print("ShiftingGears Up")
            elif self.flightStick.getRawAxis(self.flightStickShiftAxis) < -.5 or self.joyStick.getRawButton(self.joyStickShiftDown):

                self.shiftSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
                print("Shifting gears down")

        self.updateLights()

        wpilib.Timer.delay(0.005) # wait for a motor update time

    def updateLights(self):
        if self.lightIndex is not len(self.lightArr) - 1 and self.lightTimer.get() > .5:
            self.lightIndex = self.lightIndex + 1
            self.lightTimer.reset()
        if self.lightIndex is not 0 and self.lightTimer.get() > .5:
            self.lightIndex = self.lightIndex - 1
            self.lightTimer.reset()

        self.zeroLightSolenoid.set(self.lightArr[self.lightIndex][0])
        self.oneLightSolenoid.set(self.lightArr[self.lightIndex][1])
        self.twoLightSolenoid.set(self.lightArr[self.lightIndex][2])

    def testInit(self):
        self.rightWheelsPID.enable()
        self.leftWheelsPID.enable()
    def testPeriodic(self):
        #self.intake.setSpeed(self.intakeMotorSetSpeed)
        #self.topShooter.setSpeed(.7)
        self.leftWheelsPID.setSetpoint(-100)
        self.rightWheelsPID.setSetpoint(100)
        print("Left wheel distance: " + str(self.leftEncoder.getDistance()))
        print("Right wheel distance: " + str(self.rightEncoder.getDistance()))

if __name__ == "__main__":
    wpilib.run(MyRobot)
