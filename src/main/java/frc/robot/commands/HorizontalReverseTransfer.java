package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HorizontalTransferSubsystem;
import static frc.robot.Constants.HorizontalTransferConstants.*;

//Command to run the horizontal transfer mechanism in reverse.
 
public class HorizontalReverseTransfer extends Command {

    private final HorizontalTransferSubsystem htSubsystem;

    // Creates a new HorizontalReverseTransfer command.
    public HorizontalReverseTransfer(HorizontalTransferSubsystem horizontalSystem) {
        this.htSubsystem = horizontalSystem;
        addRequirements(htSubsystem);
    }

    @Override
    public void initialize() {
        htSubsystem.setHorizontalReverseTransferVelocity(HORIZONTAL_TRANSFER_REV_VELOCITY);
    }

    @Override
    public void execute() {
        // Continuous execution is handled by velocity control set in initialize
    }

    @Override
    public void end(boolean interrupted) {
        // Safely stop the rollers when the command finishes or is interrupted
        htSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted or cancelled
    }
}
