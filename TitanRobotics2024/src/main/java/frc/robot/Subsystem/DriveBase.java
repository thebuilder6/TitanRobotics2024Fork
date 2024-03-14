package frc.robot.Subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.PortMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveBase implements Subsystem
{
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  // Gains are for example purposes only - must be determined for your own robot!

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);  
  
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;

  private ModifiedEncoders leftEncoder;
  private ModifiedEncoders rightEncoder;
  private ModifiedMotors leftMotor;
  private ModifiedMotors rightMotor;
  private Gyro gyro;
  private double forward;
  private double turn;

  private double leftEncoderRate;
  private double rightEncoderRate;
  private double rightEncoderDistance;
  private double leftEncoderDistance;

  private double leftPower;
  private double rightPower;

  

  //private String motorType = "CANVictorSPX"; // This is Gyro
  //private String motorType = "CANVictorSPXDual"; // This is Janus
  private String motorType = "CANTalonDual";
  // TODO: make a better selector for the motor type

  private static DriveBase instance = null;

  public static DriveBase getInstance()
  {
    if (instance == null)
    {
      instance = new DriveBase();
    }
    return instance;
  }

  public DriveBase()
  {
    gyro = Gyro.getInstance();
    this.leftMotor = new ModifiedMotors(PortMap.FRONTLEFT.portNumber, PortMap.REARLEFT.portNumber, motorType, false);
    this.rightMotor = new ModifiedMotors(PortMap.FRONTRIGHT.portNumber, PortMap.REARRIGHT.portNumber, motorType, true);

    this.leftEncoder = new ModifiedEncoders(PortMap.LEFTENCODER_A.portNumber, PortMap.LEFTENCODER_B.portNumber,
            "E4TEncoder");
    this.rightEncoder = new ModifiedEncoders(PortMap.RIGHTENCODER_A.portNumber, PortMap.RIGHTENCODER_B.portNumber,
            "E4TEncoder");

    this.leftEncoder.setDistancePerPulse(0.155 * Math.PI / 360);
    this.rightEncoder.setDistancePerPulse(0.155 * Math.PI / 360);

    this.leftEncoder.invert(true);

    this.rightEncoder.invert(true);

    this.drive = new DifferentialDrive(leftMotor::set, rightMotor::set);
    this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getRelativeDistance(),
            rightEncoder.getRelativeDistance());
  }

  public void setRightMotorsPower(double power)
  {
    this.rightPower = power;
  }

  public void setLeftMotorsPower(double power)
  {
    this.leftPower = power;
  }

  public void drive(double forward, double turn)
  {
    this.forward = forward;
    this.turn = turn;
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftMotor.setVoltage(leftOutput + leftFeedforward);
    rightMotor.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive2(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  
  public double getLeftEncoderDistance()
  {
    return leftEncoderDistance;
  }

  public double getRightEncoderDistance()
  {
    return rightEncoderDistance;
  }

  public void log()
  {
    SmartDashboard.putNumber("leftEncoderRate", leftEncoderRate);
    SmartDashboard.putNumber("rightEncoderRate", rightEncoderRate);
    SmartDashboard.putNumber("leftEncoderDistance", leftEncoderDistance);
    SmartDashboard.putNumber("rightEncoderDistance", rightEncoderDistance);

  }

  @Override
  /* Updates the state the motors are in */
  public void update()
  {
    if (leftEncoder != null)
    {
      leftEncoderRate = this.leftEncoder.getRate();
      leftEncoderDistance = this.leftEncoder.getRelativeDistance();
    }
    else
    {
      SmartDashboardSubsystem.getInstance().error("left encoder is null");
    }
    if (rightEncoder != null)
    {
      rightEncoderRate = this.rightEncoder.getRate();
      rightEncoderDistance = this.rightEncoder.getRelativeDistance();
    }
    else
    {
      SmartDashboardSubsystem.getInstance().error("right encoder is null");
    }
    drive.arcadeDrive(forward, turn);

    this.odometry.update(gyro.getRotation2d(), leftEncoderDistance, rightEncoderDistance);
  }
}