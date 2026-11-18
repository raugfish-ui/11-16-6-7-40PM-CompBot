// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An improved AbsoluteDrive command for full field-centric driving with vector-based heading control.
 */
public class AbsoluteDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingHorizontal, headingVertical;
  private boolean initRotation = false;

  /**
   * Used to drive a swerve robot in full field-centric mode using vector heading control.
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param vX                X-translation joystick input supplier (-1 to 1).
   * @param vY                Y-translation joystick input supplier (-1 to 1).
   * @param headingHorizontal Horizontal component of the robot's heading angle.
   * @param headingVertical   Vertical component of the robot's heading angle.
   */
  public AbsoluteDrive(
      SwerveSubsystem swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingHorizontal,
      DoubleSupplier headingVertical) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    initRotation = true;
  }

  @Override
  public void execute() {
    // Sample joystick values once per cycle
    double xInput = vX.getAsDouble();
    double yInput = vY.getAsDouble();
    double hHoriz = headingHorizontal.getAsDouble();
    double hVert = headingVertical.getAsDouble();

    // Get the desired chassis speeds based on joystick inputs.
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(xInput, yInput, hHoriz, hVert);

    // Prevent sudden rotation snapping immediately after autonomous / upon init
    if (initRotation) {
      if (Math.abs(hHoriz) < 0.05 && Math.abs(hVert) < 0.05) {
        Rotation2d currentHeading = swerve.getHeading();
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, currentHeading.getSin(), currentHeading.getCos());
      }
      initRotation = false;
    }

    // Limit velocity to prevent tipping based on robot mass and chassis parameters
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(
        translation,
        swerve.getFieldVelocity(),
        swerve.getPose(),
        Constants.LOOP_TIME,
        Constants.ROBOT_MASS,
        List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());

    // Publish structured telemetry for debugging
    SmartDashboard.putNumber("AbsoluteDrive/TranslationX", translation.getX());
    SmartDashboard.putNumber("AbsoluteDrive/TranslationY", translation.getY());
    SmartDashboard.putNumber("AbsoluteDrive/OmegaRadSec", desiredSpeeds.omegaRadiansPerSecond);

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Safely stop the drivebase when the command ends or is interrupted
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
