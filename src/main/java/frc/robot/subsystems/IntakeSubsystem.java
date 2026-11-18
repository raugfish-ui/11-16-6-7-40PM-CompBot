// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  // Initializing the SPARK MAX controllers using unique IDs from IntakeConstants
  private final SparkMax pivotMotor = new SparkMax(kPivotMotorID, MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(kRollerMotorID, MotorType.kBrushless);

  public IntakeSubsystem() {
    // Subsystem constructor
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void stopPivot() {
    pivotMotor.set(0.0);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRoller() {
    rollerMotor.set(0.0);
  }

  public void setintakeSpinVelocity(double velocity) {
    // Control roller speed or velocity target
    rollerMotor.set(velocity);
  }

  public void stop() {
    // Stop both motors safely
    pivotMotor.set(0.0);
    rollerMotor.set(0.0);
  }

  public boolean isAtLowerLimit() {
    return false; // Connect to a digital input or limit switch if applicable
  }

  public boolean isAtUpperLimit() {
    return false; // Connect to a digital input or limit switch if applicable
  }
}
