package first.robot.commands.BasketballTrainingCommands;

import first.robot.subsystems.drive.Drive;
import org.wpilib.command2.SequentialCommandGroup;

public class Basketball extends SequentialCommandGroup{

    public Basketball(Drive drive)
    {
        // Add commands in the format: addCommands(new CommandName(drive, meters, seconds, etc.))
        // EX: addCommands(BasketballDriveTime(drive, ));
        addCommands();
    }
}
