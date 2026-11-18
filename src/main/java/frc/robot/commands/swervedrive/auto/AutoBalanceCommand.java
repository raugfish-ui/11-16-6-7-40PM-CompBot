// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Enhanced Auto Balance command using a PID controller with tunable dashboard metrics.
 */
public class AutoBalanceCommand extends Command {

  /**
   * Represents a command for automatically balancing the robot using the swerve drive subsystem.
   * This command utilizes the {@link SwerveSubsystem} to control the robot's movement and maintain balance.
   */
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController controller;

  // Tune these constants or move them to Constants.java
  private static final double kP = 0.025;
  private static final double kI = 0.0;
  private static final double kD = 0.001;
  
  // Maximum speed override during balancing (fraction of max speed, e.g., 0.4 = 40%)
  private static final double kMaxBalanceSpeed = 0.4;
  
  // Tolerance in degrees to consider the robot balanced
  private static final double kAngleTolerance = 1.5;

  public AutoBalanceCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    
    controller = new PIDController(kP, kI, kD);
    controller.setTolerance(kAngleTolerance);
    controller.setSetpoint(0.0);

    // Require the swerve drive subsystem
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    controller.reset();
  }

  @Override
  public void execute() {
    // Get current pitch angle from the swerve gyro
    double currentPitch = swerveSubsystem.getPitch().getDegrees();
    
    // Calculate PID output based on pitch error
    double pidOutput = controller.calculate(currentPitch, 0.0);
    
    // Clamp the output speed so the robot doesn't drive off the platform too fast
    double translationVal = MathUtil.clamp(pidOutput, -kMaxBalanceSpeed, kMaxBalanceSpeed);

    // Drive forward/backward along the pitch axis
    swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true);

    // Publish telemetry for debugging
    SmartDashboard.putNumber("AutoBalance/CurrentPitch", currentPitch);
    SmartDashboard.putNumber("AutoBalance/PIDOutput", translationVal);
    SmartDashboard.putBoolean("AutoBalance/AtSetpoint", controller.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    // Optional: You can require it to stay at setpoint for a few consecutive loops
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Lock wheels / put swerve into X-formation to hold position on the ramp
    swerveSubsystem.lock();
    // Stop any remaining motion inputs
    swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, true);
  }
}
