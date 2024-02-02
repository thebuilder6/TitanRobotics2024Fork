package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.XboxController;

public class DriverController implements Subsystem 
{
    private static DriverController instance = null;
    private static XboxController xboxController;

    public static DriverController getInstance() 
    {
        if (instance == null) {
            instance = new DriverController();
        }
        return instance;
    }

    public DriverController()
    {
        if (xboxController.getAButtonPressed())
        {

        }
    }

    @Override
    public void start() 
    {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'start'");
    }

    @Override
    public void update() 
    {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

}