package frc.robot.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntegratedMotors implements Subsystem
{
    //In the integrated motors system all motors are a motor, encoder, and PID controller

    private Encoder externalEncoder;
    private RelativeEncoder relativeEncoder;
    private SparkPIDController pidController;
    private CANSparkMax sparkMax;

    // PID coefficients
    private double kP = 0.1;
    private double kI = 1e-4;
    private double kD = 1;
    private double kIz = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private String name;

    public IntegratedMotors(int motorPort, String encoderType, int encoderPort_A, int encoderPort_B)
    {
        if (motorPort < 0)
        {
            System.err.println("Motor not assigned " + motorPort);
            sparkMax = null;
            return;
        }
        else
        {
            try
            {
                sparkMax = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
            }
            catch (Exception e)
            {
                System.err.println("Error: Motor Not Activated " + motorPort);
                sparkMax = null;
            }
        }
        if (encoderType == null || encoderPort_A < 0 || encoderPort_B < 0)
        {
            System.err.println("Encoder not Assigned " + encoderPort_A + " " + encoderPort_B);
            externalEncoder = null;
        }
        else
        {
            try
            {
                externalEncoder = new Encoder(encoderPort_A, encoderPort_B);
            }
            catch (Exception e)
            {
                System.err.println("Error: Encoder Not Activated " + encoderPort_A + " " + encoderPort_B);
                externalEncoder = null;
            }
        }
        try
        {
            relativeEncoder = sparkMax.getEncoder();
        }
        catch (Exception e)
        {
            System.err.println("Error: Relative Encoder Not Activated ");
            relativeEncoder = null;
        }
        sparkMax.restoreFactoryDefaults();

        pidController = sparkMax.getPIDController();

        if (encoderType == "External")
        {
            //
        }
        else
        {
            pidController.setFeedbackDevice(relativeEncoder);
        }

        name = "Integrated Motor " + motorPort + " ";

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidSettingsOutput();
    }

    private void pidSettingsOutput()
    {
        SmartDashboard.putNumber(name + "P Gain", kP);
        SmartDashboard.putNumber(name + "I Gain", kI);
        SmartDashboard.putNumber(name + "D Gain", kD);
        SmartDashboard.putNumber(name + "I Zone", kIz);
        SmartDashboard.putNumber(name + "Feed Forward", kFF);
        SmartDashboard.putNumber(name + "Max Output", kMaxOutput);
        SmartDashboard.putNumber(name + "Min Output", kMinOutput);
        SmartDashboard.putNumber(name + "Set Rotations", 0);
    }

    public void set(double speed)
    {
        if (sparkMax != null)
        {
            sparkMax.set(speed);// -1.0 to 1.0
        }
        else
        {
            System.err.println("Error: Motor Not Set");
        }
    }

    public void pidSettings(double p, double i, double d, double iz, double ff, double max, double min)
    {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIZone(iz);
        pidController.setFF(ff);
        pidController.setOutputRange(min, max);
    }

    public void setPosition(double position)
    {
        if (sparkMax != null)
        {
            pidController.setReference(position, CANSparkMax.ControlType.kPosition);
        }
        else
        {
            System.err.println("Error: Motor Not Set");
        }
    }

    public void tune()
    {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber(name + "P Gain", 0);
        double i = SmartDashboard.getNumber(name + "I Gain", 0);
        double d = SmartDashboard.getNumber(name + "D Gain", 0);
        double iz = SmartDashboard.getNumber(name + "I Zone", 0);
        double ff = SmartDashboard.getNumber(name + "Feed Forward", 0);
        double max = SmartDashboard.getNumber(name + "Max Output", 0);
        double min = SmartDashboard.getNumber(name + "Min Output", 0);
        double rotations = SmartDashboard.getNumber(name + "Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != kP))
        {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI))
        {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD))
        {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz))
        {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF))
        {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput))
        {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", relativeEncoder.getPosition());
    }

    public double getPosition()
    {
        return relativeEncoder.getPosition();
    }

    public double getVelocity()
    {
        return relativeEncoder.getVelocity();
    }

    public double get()
    {
        return sparkMax.get();
    }

    public double getRate()
    {
        return relativeEncoder.getVelocity();
    }

    public void setConversionFactor(double factor)
    {
        relativeEncoder.setPositionConversionFactor(factor);
        relativeEncoder.setVelocityConversionFactor(factor);

    }

    public void setVoltage(double voltage)
    {
        sparkMax.setVoltage(voltage);
    }

    public void update()
    {

    }
}
