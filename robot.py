import wpilib, ctre
from utils import ID
from subsystems import drive, rotary_joystick
from commands import turn_to_angle
# from frc import smartdashboard

class MyRobot(wpilib.TimedRobot):

   def robotInit(self):
      print("starting")

      self.rJoy = rotary_joystick.RotaryJoystick()
      self.rJoy.set_pid(0.4, 0.001, 3.2, 0) 
      self.imu_sim = 0
      #components: These are classes representing all the electrical sensors and actuators on the robot.
      self.frontLeft = ctre.WPI_TalonSRX(ID.DRIVE_LEFT_FRONT)
      self.backLeft = ctre.WPI_TalonSRX(ID.DRIVE_LEFT_BACK)

      self.frontRight = ctre.WPI_TalonSRX(ID.DRIVE_RIGHT_FRONT)
      self.backRight = ctre.WPI_TalonSRX(ID.DRIVE_RIGHT_BACK)

      self.drive_imu = ctre.PigeonIMU(self.backRight)
      self.back_limit_switch = wpilib.DigitalInput(ID.LIMIT_SWITCH_NC)

      # Might change to XBOX controller depending on it working or not.
      self.drive_stick = wpilib.Joystick(ID.DRIVE_JOYSTICK)
      self.operator_stick = wpilib.XboxController(ID.OPERATOR_JOYSTICK)

      #subsystems: These combine multiple components into a coordinated system.
      self._drive = drive.Drive(self.frontLeft, self.backLeft, self.frontRight, self.backRight, self.drive_imu)
      
      #commands: These utilize subsystems to perform autonomous routines.

   def teleopInit(self):
      self.frontLeft.setInverted(False)
      self.backLeft.setInverted(False)
      self.frontRight.setInverted(False)
      self.backRight.setInverted(False)
      
   def teleopPeriodic(self):
      # Exceptions are used to not crash robot code if in competition, 
      # but in our testing case we are raising the exceptions because we want to debug.
      try:
         angle = self.rJoy.rotary_inputs(self)

            # read joystick forwards/backwards
         left = self.rJoy.interp(self.drive_stick.getTwist())
         right = left

            # finds angle difference (delta angle) in range -180 to 180
         delta_angle = angle - self.imu_sim # self.drive_imu.getAbsoluteCompassHeading()
         delta_angle = ((delta_angle + 180) % 360) - 180

            # PID steering power limited between -12 and 12
         steer = max(-12, min(12, self.rJoy.steer_pid(delta_angle)))
         #left -= steer
         right += steer
         #self._drive.DifferentialDrive(left, right)
         #Ask Greg how to drive
         self._drive.arcadeDrive(left,steer/12)
         self.imu_sim += steer/12
         print("joy = %f, imu = %f", angle, self.imu_sim, delta_angle, steer)

         if self.drive_stick.getTop() == True:
            self._drive.arcadeDrive(self.drive_stick.getY(), self.drive_stick.getX())
         else:
            self._drive.centralDrive(self.drive_stick.getZ(), self.drive_stick.getY(), self.drive_stick.getX())
            pass
      except:
         raise
      #print("hi")
         

if __name__ == "__main__":
   wpilib.run(MyRobot)