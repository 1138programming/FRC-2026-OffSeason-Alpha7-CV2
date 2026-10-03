package first.robot.commands.IntakeCommands;

import first.robot.subsystems.Intake;

import org.wpilib.command2.Command;

import static first.robot.Constants.IntakeConstants.*;

public class IntakePivotStow extends Command {
    private final Intake intake;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakePivotStow(Intake intake) {
        
        this.intake = intake;
        
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.intakePivotToPosition(kIntakePivotZero);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntakePivot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
