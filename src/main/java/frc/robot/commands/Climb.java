package frc.robot.commands;

import static frc.robot.Constants.ClimberConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
public class Climb extends Command {
  private final ClimberSubsystem climberSubsystem;

  /** Creates a new Climb command that runs the climber at a set velocity. */
  public Climb(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climberSubsystem.setClimberVelocity(CLIMBER_VELOCITY);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the climber when the command ends or is interrupted
    climberSubsystem.setClimberVelocity(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}