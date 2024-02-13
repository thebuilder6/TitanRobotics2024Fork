package frc.robot.Subsystem;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardSubsystem implements Subsystem{
    
    private static SmartDashboardSubsystem instance = null;
    private DriveBase driveBase;
    private ClimberSubsystem climberSubsystem; 
    private AprilTagTargeting aprilTagTargeting;
    private PoseEstimation poseEstimation;
    private ClimberControl climberControl;
    List<String> errorLog = new ArrayList<>();

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
    

    public void error(String error) {
        //if error is not in errorLog
        if (!errorLog.contains(error)){
            errorLog.add(error);
        }
    }

    @Override
    public void update() {
        driveBase.log();
        climberSubsystem.log();
        aprilTagTargeting.log();
        poseEstimation.log();
        SmartDashboard.putString("Errors", errorLog.toString());
        
    }

}
