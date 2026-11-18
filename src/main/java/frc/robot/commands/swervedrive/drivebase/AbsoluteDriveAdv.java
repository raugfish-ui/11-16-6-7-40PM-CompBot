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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An advanced Swerve Control System supporting cardinal direction snap buttons and manual heading adjustment.
 */
public class AbsoluteDriveAdv extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingAdjust;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
  private boolean resetHeading = false;

  /**
   * Used to drive a swerve robot in full field-centric mode with button snapping and heading adjustment.
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            X-translation joystick input supplier (-1 to 1).
   * @param vY            Y-translation joystick input supplier (-1 to 1).
   * @param headingAdjust Heading rotation adjustment supplier (-1 to 1).
   * @param lookAway      Face the robot away from the drivers.
   * @param lookTowards   Face the robot toward the drivers.
   * @param lookLeft      Face the robot left.
   * @param lookRight     Face the robot right.
   */
  public AbsoluteDriveAdv(
      SwerveSubsystem swerve,
      DoubleSupplier vX,
      DoubleSupplier vY,
      DoubleSupplier headingAdjust,
      BooleanSupplier lookAway,
      BooleanSupplier lookTowards,
      BooleanSupplier lookLeft,
      BooleanSupplier lookRight) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    resetHeading = true;
  }

  @Override
  public void execute() {
    // Cache inputs once per cycle for consistency
    double xInput = vX.getAsDouble();
    double yInput = vY.getAsDouble();
    double adjustInput = headingAdjust.getAsDouble();
    boolean awayPressed = lookAway.getAsBoolean();
    boolean rightPressed = lookRight.getAsBoolean();
    boolean leftPressed = lookLeft.getAsBoolean();
    boolean towardsPressed = lookTowards.getAsBoolean();

    double headingX = 0;
    double headingY = 0;

    // Evaluate snap buttons (supports diagonal 45-degree combinations)
    if (awayPressed) {
      headingY = -1;
    }
    if (rightPressed) {
      headingX = 1;
    }
    if (leftPressed) {
      headingX = -1;
    }
    if (towardsPressed) {
      headingY = 1;
    }

    // Prevent sudden movement/snapping right after auto or initialization
    if (resetHeading) {
      if (headingX == 0 && headingY == 0 && Math.abs(adjustInput) < 0.05) {
        Rotation2d currentHeading = swerve.getHeading();
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(xInput, yInput, headingX, headingY);

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

    // Publish telemetry
    SmartDashboard.putNumber("AbsoluteDriveAdv/TranslationX", translation.getX());
    SmartDashboard.putNumber("AbsoluteDriveAdv/TranslationY", translation.getY());
    SmartDashboard.putNumber("AbsoluteDriveAdv/OmegaRadSec", desiredSpeeds.omegaRadiansPerSecond);

    // Drive behavior: manual rotation adjust vs. snapped heading target
    if (headingX == 0 && headingY == 0 && Math.abs(adjustInput) > 0.05) {
      resetHeading = true;
      swerve.drive(
          translation,
          (Constants.OperatorConstants.TURN_CONSTANT * -adjustInput),
          true);
    } else {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Safely stop the drivebase when the command finishes or is interrupted
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
