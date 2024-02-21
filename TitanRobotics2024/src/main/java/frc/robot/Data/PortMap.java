package frc.robot.Data;

public enum PortMap 
{
    
    XBOX_DRIVER_CONTROLLER(0), //find this number in driverstation
    XBOX_OPERATOR_CONTROLLER(1),
    CLIMBERMOTORLEFT(-4),//4
    CLIMBERMOTORRIGHT(-5),//5
    CLIMBERLEFTENCODER_A(8),
    CLIMBERLEFTENCODER_B(9),
    CLIMBERRIGHTENCODER_A(1),
    CLIMBERRIGHTENCODER_B(0),
    
    INTAKEMOTORPIVOT(-1),
    INTAKEMOTORROLLER(-2),
    INTAKEPIVOTENCODER_A(-3),
    INTAKEPIVOTENCODER_B(-4),
    INTAKEROLLERENCODER(-5),

    FRONTRIGHT(3), 
    REARRIGHT(2),
    FRONTLEFT(10),
    REARLEFT(1),
    LEFTENCODER_A(-3),
    LEFTENCODER_B(-4), //4
    RIGHTENCODER_A(-2),
    RIGHTENCODER_B(-1),

    RAMPMOTORLEFT(-1),
    RAMPMOTORRIGHT(-1);



    public int portNumber;
    private PortMap(int portValue) //constructor
    {
        this.portNumber = portValue;
    }
}