package frc.robot.Subsystem;

import frc.robot.Data.ButtonMap;

public class Control implements Subsystem 
{
    private DriveBase driveBase;
    private DriverController driverController;
    private OperatorController operatorController;
    private ClimberControl climberControl;
    private AprilTagTargeting aprilTagTargeting;
    private static Control instance = null;
    private double forward;
    private double turn;

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
        climberControl = ClimberControl.getInstance();
        aprilTagTargeting = AprilTagTargeting.getInstance();
        operatorController = OperatorController.getInstance();
    }

    public void teleopControl()
    {
        //climber controls
        if (driverController.getButton(ButtonMap.XboxA))
        {
            climberControl.top();
        }
        else if (driverController.getButton(ButtonMap.XboxB))
        {
            climberControl.bottom();
        }
        else if (driverController.getButton(ButtonMap.XboxY))
        {
            climberControl.hold();
        }
        climberControl.manualControl(operatorController.getStick(ButtonMap.XboxLEFTSTICKY), operatorController.getStick(ButtonMap.XboxRIGHTSTICKY));
        
        //drive controls
        

        forward = -driverController.getStick(ButtonMap.XboxLEFTSTICKY) * (Math.abs(driverController.getStick(ButtonMap.XboxLEFTSTICKY)));
        turn = -driverController.getStick(ButtonMap.XboxRIGHTSTICKX) * (Math.abs(driverController.getStick(ButtonMap.XboxRIGHTSTICKX)));
        if (driverController.getLeftBumper()){
            //override turn with april tag turn
            turn = aprilTagTargeting.runAprilTagXPID();
        }
        driveBase.drive(forward, turn);

        
    }

    public void start() 
    {

    }

    public void update() 
    {

    }

}