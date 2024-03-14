package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Data.ButtonMap;
import frc.robot.Data.PortMap;

public class OperatorController extends XboxController implements Subsystem
{
    private OperatorController xboxController;
    private static OperatorController instance = null;
    private boolean aReleased = true;

    public static OperatorController getInstance()
    {
        if (instance == null)
        {
            instance = new OperatorController();
        }
        return instance;
    }

    public OperatorController()
    {
        super(PortMap.XBOX_OPERATOR_CONTROLLER.portNumber);
        this.xboxController = this;
    }

    public double getStick(ButtonMap stickAxis)
    {
        if (this.xboxController != null)
        {
            try
            {
                switch (stickAxis)
                {
                    case XboxLEFTSTICKX:
                        return xboxController.getRawAxis(0);
                    case XboxLEFTSTICKY:
                        return xboxController.getRawAxis(1);
                    case XboxRIGHTSTICKX:
                        return xboxController.getRawAxis(4);
                    case XboxRIGHTSTICKY:
                        return xboxController.getRawAxis(5);
                    case XboxRIGHTTrigger:
                        return xboxController.getRawAxis(3);
                    case XboxLEFTTrigger:
                        return xboxController.getRawAxis(2);
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
        else
        {
            return 0;
        }
    }

    public boolean getButton(ButtonMap button)
    {
        if (this.xboxController != null)
        {
            try
            {
                switch (button)
                {
                    case XboxA:
                        return xboxController.getAButton();
                    case XboxB:
                        return xboxController.getBButton();
                    case XboxX:
                        return xboxController.getXButton();
                    case XboxY:
                        return xboxController.getYButton();
                    case XboxBACK:
                        return xboxController.getBackButton();
                    case XboxSTART:
                        return xboxController.getStartButton();
                    case XboxLEFTBumper:
                        return xboxController.getLeftBumper();
                    case XboxRIGHTBumper:
                        return xboxController.getRightBumper();
                    case XboxLEFTSTICKBUTTON:
                        return xboxController.getLeftStickButton();
                    case XboxRIGHTSTICKBUTTON:
                        return xboxController.getRightStickButton();
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
        else
        {
            return false;
        }
    }

    public boolean debounceA()
    {
        if (getButton(ButtonMap.XboxA) && aReleased)
        {
            aReleased = false;
            return true;
        }
        else
        {
            aReleased = !getButton(ButtonMap.XboxA);
            return false;
        }
    }

    @Override
    public void update()
    {
        // TODO Auto-generated method stub
    }

}
