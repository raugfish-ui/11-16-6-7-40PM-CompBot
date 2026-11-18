// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeLower;
import frc.robot.commands.IntakeRaise;
import frc.robot.commands.IntakeReverseSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.Shoot;
import frc.robot.commands.VerticalTransfer;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalTransferSubsystem;
import frc.robot.subsystems.flywheelShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * The core robot container managing subsystems, operator interface devices, and command bindings.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/modules"));
  private final flywheelShooterSubsystem flywheelSubsystem = new flywheelShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final VerticalTransferSubsystem vtSubsystem = new VerticalTransferSubsystem();

  // Autonomous Chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into field-relative ChassisSpeeds with angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  /**
   * Keyboard-specific drive stream for simulation testing.
   */
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Register PathPlanner Named Commands
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("shoot", new Shoot(flywheelSubsystem));
    NamedCommands.registerCommand("vertical_transfer", new VerticalTransfer(vtSubsystem));
    NamedCommands.registerCommand("intake_roller", new IntakeSpin(intakeSubsystem));
    NamedCommands.registerCommand("intake_arm_down", new IntakeLower(intakeSubsystem));
    NamedCommands.registerCommand("intake_arm_up", new IntakeRaise(intakeSubsystem));

    // Setup Autonomous Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    // Set Default Drive Commands
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    // Set Subsystem Default Commands (Single assignment per subsystem)
    flywheelSubsystem.setDefaultCommand(flywheelSubsystem.run(() -> flywheelSubsystem.stop()));
    intakeSubsystem.setDefaultCommand(intakeSubsystem.run(() -> intakeSubsystem.stop()));
    vtSubsystem.setDefaultCommand(vtSubsystem.run(() -> vtSubsystem.stop()));

    // Simulation specific bindings
    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    // Driver Controls
    if (!DriverStation.isTest()) {
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.b().and(driverXbox.a()).onTrue(Commands.runOnce(drivebase::zeroGyro));

      // Bind AbsoluteDriveAdv to Y button (Holding Y enables cardinal snapping)
      driverXbox.y().whileTrue(
          new AbsoluteDriveAdv(
              drivebase,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              driverXbox::getRightX,
              () -> driverXbox.povUp().getAsBoolean(),    // Look Away
              () -> driverXbox.povDown().getAsBoolean(),  // Look Towards
              () -> driverXbox.povLeft().getAsBoolean(),  // Look Left
              () -> driverXbox.povRight().getAsBoolean()  // Look Right
          )
      );
    }

    // Operator Controls
    operatorController.rightTrigger().whileTrue(new Shoot(flywheelSubsystem));
    operatorController.rightBumper().whileTrue(new IntakeRaise(intakeSubsystem));
    operatorController.leftBumper().whileTrue(new IntakeLower(intakeSubsystem));
    operatorController.x().whileTrue(new IntakeSpin(intakeSubsystem));
    operatorController.a().whileTrue(new IntakeReverseSpin(intakeSubsystem));
    operatorController.leftTrigger().whileTrue(new VerticalTransfer(vtSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
