package frc.robot.Subsystem;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.ButtonMap;
import frc.robot.Data.PortMap;

public class Controller extends XboxController implements Subsystem
{
    private HashMap<ButtonMap, Boolean> debounceButtons = new HashMap<ButtonMap, Boolean>();

    private String name;

    private static Controller driverInstance = null;

    public static Controller getDriverInstance()
    {
        if (driverInstance == null)
        {
            driverInstance = new Controller(PortMap.XBOX_DRIVER_CONTROLLER, "Driver Controller");
        }
        return driverInstance;
    }

    private static Controller operatorInstance = null;

    public static Controller getOperatorInstance()
    {
        if (operatorInstance == null)
        {
            operatorInstance = new Controller(PortMap.XBOX_OPERATOR_CONTROLLER, "Operator Controller");
        }
        return operatorInstance;
    }

    private Controller(PortMap port, String name)
    {
        super(port.portNumber);
        this.name = name;
        SubsystemManager.registerSubsystem(this);
    }

    public String getName()
    {
        return name;
    }

    public boolean go()
    {
        if (isConnected())
        {
            return true;
        }
        return false;

    }

    public double getStick(ButtonMap stickAxis)
    {
        try
        {
            switch (stickAxis)
            {
                case XboxLEFTSTICKX:
                    return getRawAxis(0);
                case XboxLEFTSTICKY:
                    return getRawAxis(1);
                case XboxRIGHTSTICKX:
                    return getRawAxis(4);
                case XboxRIGHTSTICKY:
                    return getRawAxis(5);
                case XboxRIGHTTrigger:
                    return getRawAxis(3);
                case XboxLEFTTrigger:
                    return getRawAxis(2);
                default:
                    return 0;
            }
        }
        catch (Exception AxisNotFound)
        {
            SmartDashboard.putString("ControllerError", "AxisNotFound");
            return 0;
        }
    }

    public boolean getButton(ButtonMap button)
    {
        try
        {
            switch (button)
            {
                case XboxA:
                    return getAButton();
                case XboxB:
                    return getBButton();
                case XboxX:
                    return getXButton();
                case XboxY:
                    return getYButton();
                case XboxBACK:
                    return getBackButton();
                case XboxSTART:
                    return getStartButton();
                case XboxLEFTBumper:
                    return getLeftBumper();
                case XboxRIGHTBumper:
                    return getRightBumper();
                case XboxLEFTSTICKBUTTON:
                    return getLeftStickButton();
                case XboxRIGHTSTICKBUTTON:
                    return getRightStickButton();
                default:
                    return false;
            }
        }
        catch (Exception ButtonNotFound)
        {
            SmartDashboard.putString("ControllerError", "ButtonNotFound");
            return false;
        }
    }

    public boolean getDebounceButton(ButtonMap button)
    {
        if (getButton(button) && debounceButtons.containsKey(button))
        {
            if (debounceButtons.get(button))
            {
                debounceButtons.put(button, false);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            debounceButtons.put(button, true);
            return false;
        }
    }

    public void update()
    {
        // TODO Auto-generated method stub
    }

    public void log()
    {
        // TODO Auto-generated method stub
    }

}