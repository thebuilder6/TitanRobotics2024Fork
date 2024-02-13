package frc.robot.Data;

public enum PortMap 
{
    
    XBOX_DRIVER_CONTROLLER(0), //find this number in driverstation
    GAMEPADFLIGHT(1), //find this number in driverstation

    FRONTRIGHT(3), 
    REARRIGHT(2),
    FRONTLEFT(0),
    REARLEFT(1),
    ARMPIVOTMOTOR(4),
    armCANID(21), 
    clawCANID(6),
    LEFTENCODER_A(3),
    LEFTENCODER_B(4),
    RIGHTENCODER_A(1),
    RIGHTENCODER_B(2);

    public int portNumber;
    private PortMap(int portValue) //constructor
    {
        this.portNumber = portValue;
    }
}