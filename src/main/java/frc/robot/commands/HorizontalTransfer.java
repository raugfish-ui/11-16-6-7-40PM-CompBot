package frc.robot.commands;

import static frc.robot.Constants.HorizontalTransferConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HorizontalTransferSubsystem;

public class HorizontalTransfer extends Command {
  private final HorizontalTransferSubsystem htSubsystem;

  /** Creates a new HorizontalTransfer command. */
  public HorizontalTransfer(HorizontalTransferSubsystem horizontalSystem) {
    this.htSubsystem = horizontalSystem;
    addRequirements(horizontalSystem);
  }

  @Override
  public void initialize() {
    htSubsystem.setHorizontalTransferVelocity(HORIZONTAL_TRANSFER_VELOCITY);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the transfer mechanism when the command ends or is interrupted
    htSubsystem.setHorizontalTransferVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
