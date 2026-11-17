package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Initialize Falcon motors using Phoenix 6 (e.g., Kraken or Falcon 500)
  private final TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);
  private final TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);

  public ShooterSubsystem() {
    // Set neutral mode (Brake or Coast)
    m_shooterLeader.setNeutralMode(NeutralModeValue.Coast);
    m_shooterFollower.setNeutralMode(NeutralModeValue.Coast);

    // If using a follower:
    // m_shooterFollower.setControl(new Follower(ShooterConstants.kShooterLeaderID, false));
  }

  public void setSpeed(double speed) {
    m_shooterLeader.set(speed);
  }

  public void stop() {
    m_shooterLeader.set(0.0);
  }
}

