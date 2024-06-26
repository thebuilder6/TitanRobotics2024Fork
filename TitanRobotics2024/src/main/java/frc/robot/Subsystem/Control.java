package frc.robot.Subsystem;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.ButtonMap;

public class Control implements Subsystem
{
    private static Control instance = null;

    private boolean useSlew = false;
    private SlewRateLimiter limiter = new SlewRateLimiter(20);
    private DriveBase driveBase;
    private Controller driverController;
    private Controller operatorController;
    private Targeting targeting;
    private ClimberControl climberControl;
    private Intake intake;
    private LimelightBack limelightBack;
    private LimelightFront limelightFront;
    private Ramp ramp;
    private IntakeControl intakeControl;

    private double forward;
    private double turn;
    private boolean inversion;
    private IntakePivot intakePivot;

    private double THRESHOLD = 0.05;

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
        SubsystemManager.registerSubsystem(this);
        driveBase = DriveBase.getInstance();
        driverController = Controller.getDriverInstance();
        operatorController = Controller.getOperatorInstance();
        targeting = Targeting.getInstance();
        intake = Intake.getInstance();
        intakePivot = IntakePivot.getInstance();
        intakeControl = IntakeControl.getInstance();
        climberControl = ClimberControl.getInstance();
        ramp = Ramp.getInstance();
        inversion = false;
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
    }

    public String getName()
    {
        return "Control";
    }

    public boolean go()
    {
        return true;
    }

    public void teleopControl()
    {
        forward = -driverController.getStick(ButtonMap.XboxLEFTSTICKY);

        turn = -driverController.getStick(ButtonMap.XboxRIGHTSTICKX);


        //Driver does not want this code on forward drive
        if (driverController.debounceSTART()) {
            useSlew = !useSlew;
            SmartDashboard.putBoolean("use slew", useSlew);
        }
        if (useSlew) {
            turn = limiter.calculate(turn);
        }
        if (driverController.getDebounceButton(ButtonMap.XboxB))
        {
            inversion = !inversion;
        }
        if (inversion)
        {
            forward = -forward;
        }


        limelightBack.setAlliance("Red");
        limelightFront.setAlliance("Red");
        // Change depending on alliance
                                       // for upcoming match.
                                       // Failure to change this will
                                       // cause you to target the
                                       // wrong AprilTags when using
                                       // lock on buttons.
        
        if (driverController.getButton(ButtonMap.XboxRIGHTBumper))
        {
           turn = targeting.noteLockOn();
        }

        if (driverController.getButton(ButtonMap.XboxLEFTBumper))
        {
            turn = targeting.aprilTagLockOn();
        }

        //driveBase.drive(forward, turn);

        driveBase.drive(forward, turn);

        climberControl();
        manipulatorControl();
    }


    private void climberControl()
    {

        if (operatorController.getButton(ButtonMap.XboxB))
        {
            climberControl.stop();
        }

        climberControl.manualControl(operatorController.getStick(ButtonMap.XboxLEFTSTICKY), operatorController.getStick(ButtonMap.XboxRIGHTSTICKY));

    }

    private void manipulatorControl()//please do not mess with the buttons, they are set to operator's preference.
    {

        if (operatorController.debounceA())
        {
            if (intakeControl.state == "disabled")
            {
                intakeControl.intaking();
            }
            else if (intakeControl.state == "up")
            {
                intakeControl.intaking();

            }
            else if (intakeControl.state == "intaking")
            {
                intakeControl.up();
            }
            else if (intakeControl.state == "up with piece")
            {
                intakeControl.score();
            }
            else if (intakeControl.state == "score piece")
            {
                intakeControl.up();
            }

        }

        if (operatorController.getButton(ButtonMap.XboxRIGHTBumper))//right bumper = intake in, pushes ramp back towards the intake
        {
            intakeControl.unClog();
            intake.manualIntakePower(0.6);

            ramp.setRamp(-0.3);
        }
        else if (operatorController.getButton(ButtonMap.XboxLEFTBumper))//left bumper = intake out, pushes ramp towards the scoring side
        {

            intakeControl.unClog();
            intake.manualIntakePower(-0.3);
            ramp.setRamp(0.3);
        }
        else if (intakeControl.state == "unClog")
        {
            intakeControl.state = "disabled";
        }
        if (operatorController.getButton(ButtonMap.XboxB))
        {
            intakeControl.disabled();
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

    public void log()
    {
        SmartDashboard.putBoolean("inversion", inversion);
    }

    public void update()
    {

    }

}