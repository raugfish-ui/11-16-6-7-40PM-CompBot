package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos { 

  /** A simple autonomous routine: spins the intake rollers for 2 seconds then stops. */
  public static Command simpleIntakeAuto(IntakeSubsystem intakeSubsystem) {
    return Commands.runEnd(
        () -> intakeSubsystem.setRollerSpeed(0.8), 
        () -> intakeSubsystem.stopRoller(), 
        intakeSubsystem
      )
      .withTimeout(2.0);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}