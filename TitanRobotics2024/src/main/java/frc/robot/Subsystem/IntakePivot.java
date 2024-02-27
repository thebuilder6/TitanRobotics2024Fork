package frc.robot.Subsystem;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Data.PortMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html

//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html

public class IntakePivot extends SubsystemBase
{

    private ModifiedMotors motor;
    private ModifiedEncoders encoder;
    private static IntakePivot instance = null;
    private ProfiledPIDController profiledPIDcontroller;

    private double kSVolts = 0;
    private double kGVolts = 0;
    private double kVVoltSecondPerDegree = 0;
    private double kAVoltSecondSquaredPerDegree = 0;
    private double kMaxVelocityDegreePerSecond = 0;
    private double kMaxAccelerationDegreePerSecSquared = 0;
    private double kP = 1;
    private double kI = 0;
    private double kD = 0;
    private double kEncoderDistancePerPulse = 360 / 2048;
    //kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    private double goal;
    private double kArmOffsetRads;
    private boolean disabled;

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
                    kSVolts, kGVolts,
                    kVVoltSecondPerDegree, kAVoltSecondSquaredPerDegree);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));

    private IntakePivot()
    {
        motor = new ModifiedMotors(PortMap.INTAKEMOTORPIVOT.portNumber, "CANSparkMax");
        encoder = new ModifiedEncoders(PortMap.INTAKEPIVOTENCODER_A.portNumber, PortMap.INTAKEPIVOTENCODER_B.portNumber, "QuadratureEncoder");
        profiledPIDcontroller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(
                        kMaxVelocityDegreePerSecond,
                        kMaxAccelerationDegreePerSecSquared));
        encoder.setDistancePerPulse(kEncoderDistancePerPulse);
        disabled = true;
    }

    public static IntakePivot getInstance()
    {
        if (instance == null)
        {
            instance = new IntakePivot();
        }
        return instance;
    }

    public void setPosition(double position)
    {
        goal = position + kArmOffsetRads;
    }

    public void setDisabled(boolean disabled)
    {
        this.disabled = disabled;
    }

    private void control()
    {
        profiledPIDcontroller.setGoal(goal + kArmOffsetRads);
        motor.setVoltage(profiledPIDcontroller.calculate(encoder.getDistance()) + m_feedforward.calculate(profiledPIDcontroller.getSetpoint().position, profiledPIDcontroller.getSetpoint().velocity));
    }

    public void update()
    {
        if (motor != null)
        {
            if (disabled)
            {
                motor.setVoltage(0);
            }
            else
            {
                control();
            }
        }
    }

    public void log()
    {
        SmartDashboard.getNumber("IntakePivotGoal", goal);
        SmartDashboard.getNumber("IntakePivotPosition", encoder.getDistance());
        SmartDashboard.getNumber("IntakePivotVelocity", encoder.getRate());
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
