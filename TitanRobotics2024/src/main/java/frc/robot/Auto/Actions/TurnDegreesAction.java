package frc.robot.Auto.Actions;

import frc.robot.Subsystem.DriveBase;
import frc.robot.Subsystem.Gyro;
import frc.robot.Subsystem.PositionEstimation;

import javax.swing.text.Position;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TurnDegreesAction implements Actions 
{
    Timer timer;
    private double currentDegrees = 0;
    private double desiredDegrees;
    private double targetDegrees;
    private double turn;
    private double toleranceDegrees = 0.0025;
    private double endAfterSeconds;

    private DriveBase driveBase;
    private Gyro gyro;
    private PositionEstimation position;

    private PIDController PID;
    private double kp = 0.01;
    private double ki = 0.011;
    private double kd = 0.002;

    public TurnDegreesAction(double degrees, double seconds)
    {
        driveBase = DriveBase.getInstance();
        //gyro = Gyro.getInstance();
        position = PositionEstimation.getInstance();
        endAfterSeconds = seconds;
        desiredDegrees = degrees;
        PID = new PIDController(kp, ki, kd);
        PID.enableContinuousInput(-180, 180);
        

    }

    @Override
    public void start()
    {
        timer = new Timer();
        timer.start();
        //currentDegrees = gyro.getAngleDegrees();
        currentDegrees = position.getAngle();
        targetDegrees = (currentDegrees + desiredDegrees);
        PID.setSetpoint(targetDegrees);
        PID.setTolerance(toleranceDegrees);
       // System.out.println(targetDegrees);
       SmartDashboard.putString( "Current Action", "TurnDegreesAction Started");
    }

 

    @Override
    public void update()
    {
        turn = PID.calculate(position.getAngle());
        driveBase.drive(0, turn);
        //System.out.println(gyro.getAngleDegrees());
        SmartDashboard.putNumber("targetDegrees", targetDegrees);
        SmartDashboard.putNumber("turn", turn);
        //System.out.println(turn);
        
    }

    @Override
    public boolean isFinished()
    {
        if (timer.get() >= endAfterSeconds)
        {
            System.out.println("ended properly");
            return true;
            
        }
        else 
        {
            return false;
        }
    }
    

    @Override
    public void done()
    {
        SmartDashboard.putString( "Current Action", "TurnDegreesAction Ended");
        driveBase.drive(0, 0);
    }
}
