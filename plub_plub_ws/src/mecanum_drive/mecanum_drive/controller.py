from __future__ import division

class MotorCommand:
    """Holds motor control commands for robot."""

    def __init__(self):
        # Initialise desired wheel speeds (ticks/s) to zero
    """Holds motor control commands for a differential-drive robot.
    """

    def __init__(self):
        self.frontLeft = 0
        self.frontRight = 0
        self.rearLeft = 0
        self.rearRight = 0


class Controller:
    """Converts target linear / angular motion to individual wheel speeds."""

    def __init__(self):
        # Start with a very high limit so speeds are effectively unbounded
        self.maxMotorSpeed = 10_000_000  # ticks/s

    def getSpeeds(self, linearXSpeed, linearYSpeed, angularSpeed):
        # Local aliases for readability
        WHEEL_SEPARATION_WIDTH = self.wheelSeparation
        WHEEL_SEPARATION_LENGTH = self.wheelSeparationLength

        speeds = MotorCommand()

        # Inverse kinematics for mecanum four‑wheel drive
        speeds.frontLeft = self.ticksPerMeter * (
            linearXSpeed - linearYSpeed -
            (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angularSpeed
        )
        speeds.frontRight = self.ticksPerMeter * (
            linearXSpeed + linearYSpeed +
            (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angularSpeed
        )
        speeds.rearLeft = self.ticksPerMeter * (
            linearXSpeed + linearYSpeed -
            (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angularSpeed
        )
        speeds.rearRight = self.ticksPerMeter * (
            linearXSpeed - linearYSpeed +
            (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * angularSpeed
        )

        # Scale down uniformly if any wheel exceeds the allowed maximum
        max_requested = max(
            speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight
        )
        if max_requested > self.maxMotorSpeed:
            factor = self.maxMotorSpeed / max_requested
        

class Controller:
    """Determines motor speeds to accomplish a desired motion.
    """

    def __init__(self):
        # Set the max motor speed to a very large value so that it
        # is, essentially, unbound.
        self.maxMotorSpeed = 10000000 # ticks/s

    def getSpeeds(self, linearXSpeed, linearYSpeed, angularSpeed):
    
    
        # print(linearXSpeed,linearYSpeed,angularSpeed)
    
        WHEEL_SEPARATION_WIDTH = self.wheelSeparation
        WHEEL_SEPARATION_LENGTH = self.wheelSeparationLength
        
        speeds = MotorCommand()
        
        speeds.frontLeft = self.ticksPerMeter * (linearXSpeed - linearYSpeed - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.frontRight = self.ticksPerMeter * (linearXSpeed + linearYSpeed + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.rearLeft = self.ticksPerMeter * (linearXSpeed + linearYSpeed - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        speeds.rearRight = self.ticksPerMeter * (linearXSpeed - linearYSpeed + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularSpeed)
        
        # print(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight)
        

        # Adjust speeds if they exceed the maximum.
        if max(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight) > self.maxMotorSpeed:
            factor = self.maxMotorSpeed / max(speeds.frontLeft, speeds.frontRight, speeds.rearLeft, speeds.rearRight)
            speeds.frontLeft *= factor
            speeds.frontRight *= factor
            speeds.rearLeft *= factor
            speeds.rearRight *= factor

        if speeds.frontLeft != 0 and abs(speeds.frontLeft) < self.minMotorSpeed:
            speeds.frontLeft =  self.minMotorSpeed if speeds.frontLeft > 0 else -self.minMotorSpeed
        if speeds.frontRight != 0 and abs(speeds.frontRight) < self.minMotorSpeed:
            speeds.frontRight = self.minMotorSpeed if speeds.frontRight > 0 else -self.minMotorSpeed
        if speeds.rearLeft != 0 and abs(speeds.rearLeft) < self.minMotorSpeed:
            speeds.rearLeft =   self.minMotorSpeed if speeds.rearLeft > 0 else -self.minMotorSpeed
        if speeds.rearRight != 0 and abs(speeds.rearRight) < self.minMotorSpeed:
            speeds.rearRight =  self.minMotorSpeed if speeds.rearRight > 0 else -self.minMotorSpeed


        # Cast to int before returning, as required by downstream code
        speeds.frontLeft = int(speeds.frontLeft)
        speeds.frontRight = int(speeds.frontRight)
        speeds.rearLeft = int(speeds.rearLeft)
        speeds.rearRight = int(speeds.rearRight)
        return speeds

    # Simple setter methods follow
    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation 
        
    def setWheelSeparationLength(self, separation):
        self.wheelSeparationLength = separation

    def setMaxMotorSpeed(self, limit):
        self.maxMotorSpeed = limit

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks
