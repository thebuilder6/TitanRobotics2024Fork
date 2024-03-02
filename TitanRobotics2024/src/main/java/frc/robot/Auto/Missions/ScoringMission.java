package frc.robot.Auto.Missions;

import frc.robot.Auto.AutoMissionEndedException;
import frc.robot.Auto.Actions.DriveForDistanceAction;
import frc.robot.Auto.Actions.TurnDegreesAction;
import frc.robot.Auto.Actions.RampAction;

//none of the number values are for certain, they are just placeholders

public class ScoringMission extends MissionBase
{
     @Override
    protected void routine() throws AutoMissionEndedException 
    // should work from alliance wall, might need to be slightly adjusted for aiode, distance measurements are in meters
    {
        runAction(new DriveForDistanceAction(1.7, 3));  
        //runAction(new DriveForTimeAction(1, 0.5)
        runAction(new TurnDegreesAction(-77, 3));
        runAction(new DriveForDistanceAction(2, 3));
        // runaction(new IntakeAction(3)); needed? Depends on where the note starts preloaded in the robot
        runAction(new RampAction(3, 0.3));
    }
}

