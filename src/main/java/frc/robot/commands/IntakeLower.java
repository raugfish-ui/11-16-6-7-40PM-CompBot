package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeLower extends Command {
  private final IntakeSubsystem m_intake;

  public IntakeLower(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    // Run pivot downward (adjust speed and sign based on your gearing)
    m_intake.setPivotSpeed(-0.3);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopPivot();
  }

  @Override
  public boolean isFinished() {
    // Stops automatically if using a limit switch/encoder; otherwise runs while held
    return m_intake.isAtLowerLimit();
  }
}
