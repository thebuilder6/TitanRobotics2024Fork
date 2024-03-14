package frc.robot.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager
{
    private static List<Subsystem> subsystems = new ArrayList<>();

    public static void registerSubsystem(Subsystem subsystem)
    {
        subsystems.add(subsystem);
    }

    public static void updateAllSubsystems()
    {
        for (Subsystem subsystem : subsystems)
        {
            subsystem.update();
        }
    }

    public static void logAllSubsystems()
    {
        for (Subsystem subsystem : subsystems)
        {
            subsystem.log();
        }
    }

    public static List<Subsystem> getSubsystems()
    {
        return subsystems;
    }
}
