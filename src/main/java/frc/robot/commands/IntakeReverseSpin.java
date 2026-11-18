package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Command to spin the intake rollers in reverse to eject game pieces.
 */
public class IntakeReverseSpin extends Command {

    private final IntakeSubsystem intakeSubsystem;

    /** Creates a new IntakeReverseSpin command. */
    public IntakeReverseSpin(IntakeSubsystem intakeSystem) {
        this.intakeSubsystem = intakeSystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setintakeSpinVelocity(INTAKE_SPIN_REV_VELOCITY);
    }

    @Override
    public void execute() {
        // Velocity target is maintained by the motor controller
    }

    @Override
    public void end(boolean interrupted) {
        // Stop only the roller so pivot motion isn't disrupted if running concurrently
        intakeSubsystem.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
