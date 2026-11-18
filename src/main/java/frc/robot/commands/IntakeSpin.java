// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

/**
 * Command to spin the intake rollers forward to intake game pieces.
 */
public class IntakeSpin extends Command {

  private final IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeSpin command. */
  public IntakeSpin(IntakeSubsystem intakeSystem) {
    this.intakeSubsystem = intakeSystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setintakeSpinVelocity(INTAKE_SPIN_VELOCITY);
  }

  @Override
  public void execute() {
    // Velocity target is maintained by the SparkMax closed-loop controller
  }

  @Override
  public void end(boolean interrupted) {
    // Safely stop the intake rollers when the command finishes or is interrupted
    intakeSubsystem.stopRoller();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted or cancelled
  }
}
