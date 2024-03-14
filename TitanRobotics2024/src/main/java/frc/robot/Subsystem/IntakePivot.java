package frc.robot.Subsystem;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.PortMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html

//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html

public class IntakePivot implements Subsystem
{
    private static IntakePivot instance = null;
    private ModifiedMotors pivotMotor;
    private ModifiedEncoders pivotEncoder;
    private ProfiledPIDController pivotProfiledPIDController;
    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kSVolts = 0.0;
    private double kGVolts = 0.0;
    private double kVVolts = 0.0;
    private double kAVolts = 0.0;
    private double currentPosition;

    private double upGoal = 386.0;
    private double downGoal = 205.0;

    private double maxVelocity;
    private double maxAcceleration;
    private double encoderDistancePerRotation = 360.0;
    private double encoderPositionOffset = 0.0;
    private boolean disabled;
    private double goal;
    private double startingOffset = 0.0;

    public String intakeState = "disabled";

    private final ArmFeedforward feedforward = new ArmFeedforward(kSVolts, kGVolts, kVVolts, kAVolts);
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

    public static IntakePivot getInstance()
    {
        if (instance == null)
        {
            instance = new IntakePivot();
        }
        return instance;
    }

    public IntakePivot()
    {
        pivotMotor = new ModifiedMotors(PortMap.INTAKEMOTORPIVOT.portNumber, "CANSparkMax");
        pivotEncoder = new ModifiedEncoders(PortMap.INTAKEPIVOTENCODER.portNumber, encoderPositionOffset, "DutyCycleEncoder");
        if (pivotMotor == null)
        {
            System.out.println("Pivot null");
        }
        pivotProfiledPIDController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

        pivotEncoder.setDistancePerRotation(encoderDistancePerRotation);
        disabled = true;
    }

    private void control()
    {
        pivotProfiledPIDController.setGoal(goal + startingOffset);

        pivotMotor.setVoltage(0.3 * (pivotProfiledPIDController.calculate(currentPosition) + feedforward.calculate(pivotProfiledPIDController.getSetpoint().position, pivotProfiledPIDController.getSetpoint().velocity)));
    }

    public void disabled()
    {
        this.intakeState = "disabled";
    }

    public void up()
    {
        this.intakeState = "up";
    }

    public void down()
    {
        this.intakeState = "down";
    }

    private void IntakeStateProcess()
    {
        switch (intakeState)
        {
            case "disabled":
                setDisabled(true);
                break;
            case "up":
                goal = upGoal;
                setDisabled(false);
                break;
            case "down":
                goal = downGoal;
                setDisabled(false);
                break;
            default:
                break;
        }
    }

    public void setTargetPosition(double position)
    {

        goal = position;
        this.disabled = false;
    }

    public void setDisabled(boolean disabled)
    {
        this.disabled = disabled;
    }
    
    public void manualPivotPower(double power)
    {
        pivotMotor.setVoltage(power);
    }
    
    public void log()
    {
        SmartDashboard.putNumber("pivotAbsoluteEncoder", currentPosition);
    }


    // Create a new SysId routine for characterizing the drive.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                                    // Tell SysId how to plumb the driving voltage to the motors.
                                    (Measure<Voltage> volts) -> {
                                        motor.setVoltage(volts.in(Volts));
                                    },
                                    // Tell SysId how to record a frame of data for each motor on the mechanism being
                                    // characterized.
                                    log -> {
                                        // Record a frame for themotors. 
                                        log.motor("intakepivot")
                                                        .voltage(
                                                                        m_appliedVoltage.mut_replace(
                                                                                        motor.get() * RobotController.getBatteryVoltage(), Volts))
                                                        .angularPosition(m_angle.mut_replace(encoder.getDistance(), Degrees))
                                                        .angularVelocity(m_velocity.mut_replace(encoder.getRate(), DegreesPerSecond));
                                    },
                                    // Tell SysId to make generated commands require this subsystem, suffix test state in
                                    // WPILog with this subsystem's name ("drive")
                                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutine.dynamic(direction);
    }

}

    public void update()
    {
        if (pivotEncoder != null)
        {
            currentPosition = pivotEncoder.getAbsolutePosition();
            if (currentPosition <= 120)
            {
                currentPosition += 360;
            }
            IntakeStateProcess();
        }
        if (!disabled)
        {
            control();
        }
        else
        {
            pivotMotor.setVoltage(0.0);
        }
    }

}
