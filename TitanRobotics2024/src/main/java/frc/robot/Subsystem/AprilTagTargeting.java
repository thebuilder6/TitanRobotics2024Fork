package frc.robot.Subsystem;

import frc.robot.ExternalLibraries.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystem.Limelight;
import frc.robot.Subsystem.DriveBase;

public class AprilTagTargeting implements Subsystem // This class contains functions for finding and
                                                    // locking onto elements of the field using
                                                    // their April Tags.
{
    private static AprilTagTargeting instance = null;

    public static AprilTagTargeting getInstance() {
        if (instance == null)
        {
            instance = new AprilTagTargeting();
        }
        return instance;
    }

    private PIDController aprilTagXPID = new PIDController(1, 0, 0);
    private PIDController aprilTagAPID = new PIDController(1, 0, 0);

    String alliance = "red";
    String target = "ALL";


    public void setAlliance(String alliance) {
        this.alliance = alliance;
    }

    Limelight limelight;
    DriveBase driveBase;

    public AprilTagTargeting() {
        limelight = Limelight.getInstance();
        driveBase = DriveBase.getInstance();
    }

    public void setTarget(String target) {
        this.target = target;
    }

    public void runAprilTagXPID() {
        if (target.equals("ALL") || target.equals(findTagName())) {
            driveBase.drive(0, (aprilTagXPID.calculate(limelight.x, 0)) / 25);
        }
        
    }

    public void runAprilTagZPID() {
        driveBase.drive((aprilTagAPID.calculate(limelight.z, 0)) / 25,0 ); // zero is a placeholder
    }

    public String findTagName() {
        if (!limelight.getLimelightState().equals("TRACKING")) {
            return "Not Tracking";
        }
        switch (limelight.getId()) {
            case 11, 12, 13 -> {
                return alliance.equals("red")  ? "Stage" : "Opponent's Stage";
            }
            case 14, 15, 16 -> {
                return alliance.equals("blue") ? "Stage" : "Opponent's Stage";
            }
            case 5 -> {
                return alliance.equals("red")  ? "Amp" : "Opponent's Amp";
            }
            case 6 -> {
                return alliance.equals("blue") ? "Amp" : "Opponent's Amp";
            }
            case 9, 10 -> {
                return alliance.equals("red")  ? "Source" : "Opponent's Source";
            }
            case 1, 2 -> {
                return alliance.equals("blue") ? "Source" : "Opponent's Source";
            }
            case 3, 4 -> {
                return alliance.equals("red")  ? "Speaker" : "Opponent's Speaker";
            }
            case 7, 8 -> {
                return alliance.equals("blue") ? "Speaker" : "Opponent's Speaker";
            }
            default -> {
                return "Unknown";
            }
        }
    }

    public void log() {
        SmartDashboard.putString("AprilTag Target", findTagName());
    }

    public void update() {}
}
