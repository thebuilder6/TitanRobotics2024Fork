package frc.robot.Subsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.PortMap;

public class Gyro implements Subsystem
{
    private static Gyro instance = null;

    double yAxisRate;
    double xAxisRate;
    double pitchAngleDegrees = 0;
    double rollAngleDegrees = 0;
    double yawAngleDegrees = 0;
    double angleDegrees = 0;

    AHRS ahrs;

    public static Gyro getInstance() 
    {
        if (instance == null) 
        {
            instance = new Gyro();
        }
        return instance;
    }

    public Gyro() 
    {
        try 
        {
            ahrs = new AHRS(SPI.Port.kMXP);
        }
        catch (Exception e)
        {
            System.out.println("Could not get ahrs; did you drop metal shavings in the RoboRio again?");
        }
        
        ahrs.reset();
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

     public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }

    public void reset()
    {
        //ahrs.zeroYaw();-+
        
    }

    @Override
    public void update() 
    {
        // TODO Auto-generated method stub
    }
}
