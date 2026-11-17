package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Initializing the SPARK MAX controllers for a brushless motor (like a NEO or NEO 550)
  private final SparkMax m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
  private final SparkMax m_rollerMotor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

  public IntakeSubsystem() {
    // You can add factory resets or motor configuration safety calls here if needed
  }

  public void setPivotSpeed(double speed) {
    m_pivotMotor.set(speed);
  }

  public void stopPivot() {
    m_pivotMotor.set(0.0);
  }

  public void setRollerSpeed(double speed) {
    m_rollerMotor.set(speed);
  }

  public void stopRoller() {
    m_rollerMotor.set(0.0);
  }

  public boolean isAtLowerLimit() {
    return false; // Connect to a digital input or limit switch if applicable
  }

  public boolean isAtUpperLimit() {
    return false; // Connect to a digital input or limit switch if applicable

  }

// Add this method to the IntakeSubsystem class

public void stop() {

    // Add logic to stop the intake subsystem

}

public void setintakeSpinVelocity(double intakeSpinVelocity) {
  // TODO Auto-generated method stub
  throw new UnsupportedOperationException("Unimplemented method 'setintakeSpinVelocity'");
}

}
