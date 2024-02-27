package frc.robot.Subsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Data.ButtonMap;

public class Control implements Subsystem
{
    private static Control instance = null;

    private DriveBase driveBase;
    private DriverController driverController;
    private OperatorController operatorController;
    private Targeting targeting;
    private ClimberControl climberControl;
    private Intake intake;
    private Limelight limelight;
    private Ramp ramp;
    private IntakePivot intakePivot;

    private double rampspeed;
    private double forward;
    private double turn;

    private double THRESHOLD = 0.05;
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private double kMaxSpeed = 3.0;

    private double kMaxRotSpeed = 2 * Math.PI;

    public static Control getInstance()
    {
        if (instance == null)
        {
            instance = new Control();
        }
        return instance;
    }

    public Control()
    {
        driveBase = DriveBase.getInstance();
        driverController = DriverController.getInstance();
        operatorController = OperatorController.getInstance();
        targeting = Targeting.getInstance();
        limelight = Limelight.getInstance();
        intake = Intake.getInstance();
        climberControl = ClimberControl.getInstance();
        ramp = Ramp.getInstance();
        intakePivot = IntakePivot.getInstance();
    }

    public void teleopControl()
    {
        forward = -driverController.getStick(ButtonMap.XboxLEFTSTICKY) * (Math.abs(driverController.getStick(ButtonMap.XboxLEFTSTICKY)));
        turn = -driverController.getStick(ButtonMap.XboxRIGHTSTICKX);
        forward = m_speedLimiter.calculate(forward) * kMaxSpeed;
        turn = m_rotLimiter.calculate(turn) * kMaxRotSpeed;
        //limelightControl();

        //driveBase.drive(forward, turn);

        //climberControl();
        // manipulatorControl();

        intakePivot.setDisabled(false);
        intakePivotSysID();
    }

    private void limelightControl()
    {
        targeting.setAlliance("blue"); // Change depending on alliance
                                       // for upcoming match.
                                       // Failure to change this will
                                       // cause you to target the
                                       // wrong AprilTags when using
                                       // lock on buttons.
        if (driverController.debounceSTART())
        {
            System.out.println("pressed");
            if (limelight.getPipeline() == 0)
            {
                limelight.setPipeline(1);
            }
            else
            {
                limelight.setPipeline(0);
            }
        }

        if (limelight.getPipeline() == 0)
        {
            if (driverController.getButton(ButtonMap.XboxX))
            {
                targeting.setTarget("Amp");
                turn = targeting.aprilTaglockOn();
                System.out.println("Locking on to Amp");
            }

            if (driverController.getButton(ButtonMap.XboxA))
            {
                targeting.setTarget("Source");
                turn = targeting.aprilTaglockOn();
                System.out.println("Locking on to Source");
            }

            if (driverController.getButton(ButtonMap.XboxB))
            {
                targeting.setTarget("Stage");
                turn = targeting.aprilTaglockOn();
                System.out.println("Locking on to Stage");
            }

            else
            {
                targeting.setTarget("none");
            }
        }

        if (limelight.pipeline == 1)
        {
            if (driverController.getButton(ButtonMap.XboxY))
            {
                turn = targeting.otherLockOn();
                System.out.println("Locking On to Note");
            }
        }

        if (driverController.getButton(ButtonMap.XboxRIGHTBumper))
        {
            forward = targeting.follow();
            turn = targeting.otherLockOn();
        }
    }

    private void climberControl()
    {

        // if (operatorController.getButton(ButtonMap.XboxY))
        // {
        //     climberControl.top();
        // }

        // if (operatorController.getButton(ButtonMap.XboxX))
        // {
        //     climberControl.bottom();
        // }

        // if (operatorController.getButton(ButtonMap.XboxB))
        // {
        //     climberControl.stop();
        // }

        climberControl.manualControl(operatorController.getStick(ButtonMap.XboxLEFTSTICKY), operatorController.getStick(ButtonMap.XboxRIGHTSTICKY));

    }

    private void manipulatorControl()
    {
        if (operatorController.getButton(ButtonMap.XboxRIGHTBumper))
        {
            intake.manualIntakePower(0.3);
            intake.manualPivotPower(0);
            ramp.setRamp(0);
        }
        else if (operatorController.getButton(ButtonMap.XboxLEFTBumper))
        {
            intake.manualIntakePower(-0.3);
            intake.manualPivotPower(0);
            ramp.setRamp(0.3);
        }
        else if (operatorController.getButton(ButtonMap.XboxY))
        {
            intake.manualIntakePower(0);
            intake.manualPivotPower(0.1);
            ramp.setRamp(0);
        }
        else if (operatorController.getButton(ButtonMap.XboxA))
        {
            intake.manualIntakePower(0);
            intake.manualPivotPower(-0.1);
            ramp.setRamp(0);
        }
        else
        {
            intake.manualIntakePower(0);
            intake.manualPivotPower(0);
            ramp.setRamp(0);
        }
    }

    public void intakePivotSysID()
    {
        if (driverController.getButton(ButtonMap.XboxA))
        {
            intakePivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        }
        if (driverController.getButton(ButtonMap.XboxB))
        {
            intakePivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
        }
        if (driverController.getButton(ButtonMap.XboxX))
        {
            intakePivot.sysIdDynamic(SysIdRoutine.Direction.kForward);
        }
        if (driverController.getButton(ButtonMap.XboxY))
        {
            intakePivot.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        }
    }

    public void start()
    {

    }

    public void update()
    {

    }

}