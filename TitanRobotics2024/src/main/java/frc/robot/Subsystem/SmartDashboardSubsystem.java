package frc.robot.Subsystem;

public class SmartDashboardSubsystem implements Subsystem{
    
    private static SmartDashboardSubsystem instance = null;
    private DriveBase driveBase;
    private ClimberSubsystem climberSubsystem; 
    private AprilTagTargeting aprilTagTargeting;
    private PoseEstimation poseEstimation;
    private ClimberControl climberControl;

    public static SmartDashboardSubsystem getInstance() {
        if (instance == null) {
            instance = new SmartDashboardSubsystem();
        }
        return instance;
    }

    private SmartDashboardSubsystem() {
        aprilTagTargeting = AprilTagTargeting.getInstance();
        poseEstimation = PoseEstimation.getInstance();
        driveBase = DriveBase.getInstance();
        climberControl = ClimberControl.getInstance();
    }
    
    @Override
    public void update() {
        driveBase.log();
        climberSubsystem.log();
        aprilTagTargeting.log();
        poseEstimation.log();
        
    }

}
