package frc.robot.Commands;

import frc.robot.Subsystems.SwerveDrive;

public class AutonomousCommands {
    private final SwerveDrive s_drive = SwerveDrive.getInstance();

    private static AutonomousCommands instance;

    public static AutonomousCommands getInstance() {
        if(instance == null) {
            instance = new AutonomousCommands();
        }
        return instance;
    }

    // place command sequences here
}
