package frc.robot.Subsystem;

import com.kauailabs.navx.frc.AHRS; //https://dev.studica.com/releases/2024/NavX.json
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IMU implements Subsystem
{

  private static IMU instance = null;
  AHRS ahrs;
  SmartDashboardSubsystem smartDashboardSubsystem;

  double yAxisRate;
  double xAxisRate;
  double pitchAngleDegrees = 0;
  double rollAngleDegrees = 0;
  double yawAngleDegrees = 0;
  double angleDegrees = 0;

  public static IMU getInstance()
  {
    if (instance == null)
    {
      instance = new IMU();
    }
    return instance;
  }

  public IMU()
  {
    smartDashboardSubsystem = SmartDashboardSubsystem.getInstance();
    try
    {
      ahrs = new AHRS(SPI.Port.kMXP);
    }
    catch (Exception e)
    {
      System.out.println(
              "Could not get ahrs; did you drop metal shavings in the RoboRio again?");
      smartDashboardSubsystem.error(
              "Could not get ahrs; did you drop metal shavings in the RoboRio again?");
    }
    SubsystemManager.registerSubsystem(this);
    ahrs.reset();
  }

  public String getName()
  {
    return "IMU";
  }

  public boolean go()
  {
    if (ahrs.isConnected())
    {
      return true;
    }
    return false;
  }

  public void log()
  {
    SmartDashboard.putNumber("GyroYaw", ahrs.getYaw());
    SmartDashboard.putNumber("GyroAngle", ahrs.getAngle());
  }

  public double getRollDegrees()
  {
    rollAngleDegrees = ahrs.getRoll();
    return rollAngleDegrees;
  }

  public double getYawDegrees()
  {
    yawAngleDegrees = ahrs.getYaw();
    return yawAngleDegrees;
  }

  public double getPitchDegrees()
  {
    pitchAngleDegrees = ahrs.getPitch();
    return pitchAngleDegrees;
  }

  public double getAngleDegrees()
  {
    return ahrs.getAngle();
  }

  public double getAngleRate()
  {
    return ahrs.getRate();
  }

  public Rotation2d getRotation2d()
  {
    return ahrs.getRotation2d();
  }

  public void reset()
  {

  }

  @Override
  public void update()
  {
    // TODO Auto-generated method stub
  }
}
