package frc.robot.Data;

public enum PortMap 
{
    
    XBOX_DRIVER_CONTROLLER(1), //find this number in driverstation
    XBOX_OPERATOR_CONTROLLER(2),

    CLIMBERMOTORLEFT(4),
    CLIMBERMOTORRIGHT(5),
    CLIMBERLEFTENCODER_A(4),
    CLIMBERLEFTENCODER_B(4),
    CLIMBERRIGHTENCODER_A(5),
    CLIMBERRIGHTENCODER_B(5),
    FRONTRIGHT(3), 
    REARRIGHT(2),
    FRONTLEFT(0),
    REARLEFT(1),
    LEFTENCODER_A(3),
    LEFTENCODER_B(-1), //4
    RIGHTENCODER_A(2),
    RIGHTENCODER_B(1);


    public int portNumber;
    private PortMap(int portValue) //constructor
    {
        this.portNumber = portValue;
    }
}