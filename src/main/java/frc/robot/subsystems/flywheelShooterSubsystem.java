package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class flywheelShooterSubsystem extends SubsystemBase {
  private final SparkFlex flywheelLeaderMotor;
  private final SparkFlex flywheelFollowerMotor;
  private final RelativeEncoder flywheelEncoder;
  private final SparkClosedLoopController flywheelClosedLoopController;

  /** Creates a new flywheelShooterSubsystem. */
  public flywheelShooterSubsystem() {
    flywheelLeaderMotor = new SparkFlex(FLYWHEEL_LEADER_ID, MotorType.kBrushless);
    flywheelFollowerMotor = new SparkFlex(FLYWHEEL_FOLLOWER_ID, MotorType.kBrushless);
    
    flywheelClosedLoopController = flywheelLeaderMotor.getClosedLoopController();
    flywheelEncoder = flywheelLeaderMotor.getEncoder();

    // Configure Leader Motor
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig.inverted(false);
    flywheelConfig.idleMode(IdleMode.kCoast);
    flywheelConfig.smartCurrentLimit(FLYWHEEL_SHOOTER_CURRENT_LIMIT);

    flywheelConfig.encoder.positionConversionFactor(1.0);
    flywheelConfig.encoder.velocityConversionFactor(1.0);
    
    flywheelConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(FLYWHEEL_MOTOR_P, ClosedLoopSlot.kSlot0)
        .i(FLYWHEEL_MOTOR_I, ClosedLoopSlot.kSlot0)
        .d(FLYWHEEL_MOTOR_D, ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

    flywheelConfig.softLimit
        .forwardSoftLimit(FLYWHEEL_MOTOR_FWD_LIMIT)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(FLYWHEEL_MOTOR_REV_LIMIT)
        .reverseSoftLimitEnabled(false);

    flywheelLeaderMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure Follower Motor
    SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();
    flywheelFollowerConfig.inverted(false);
    flywheelFollowerConfig.idleMode(IdleMode.kCoast);
    flywheelFollowerConfig.smartCurrentLimit(FLYWHEEL_SHOOTER_CURRENT_LIMIT);
    flywheelFollowerConfig.follow(FLYWHEEL_LEADER_ID);
    
    flywheelFollowerMotor.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize dashboard target value
    SmartDashboard.putNumber("Flywheel/TargetVoltage", SHOOTER_FLYWHEEL_VOLTAGE);
  }

  /** Sets the voltage of the flywheel shooter. */
  public void setFlywheelVoltage(double voltage) {
    flywheelLeaderMotor.setVoltage(voltage);
  }

  /** Legacy alias for backward compatibility with existing commands. */
  public void setflywheelShooter(double voltage) {
    setFlywheelVoltage(voltage);
  }

  /** Runs the flywheel using closed-loop velocity control. */
  public void setFlywheelVelocity(double velocity) {
    flywheelClosedLoopController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  /** Legacy alias for backward compatibility with Shoot.java. */
  public void setflywheelShooterVelocity(double velocity) {
    setFlywheelVelocity(velocity);
  }

  /** Gets the current velocity of the flywheel. */
  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  /** Legacy alias for backward compatibility with existing commands. */
  public double getflywheelShooterVelocity() {
    return getFlywheelVelocity();
  }

  /** Gets the current position of the flywheel encoder. */
  public double getFlywheelPosition() {
    return flywheelEncoder.getPosition();
  }

  /** Stops the flywheel motors. */
  public void stop() {
    flywheelLeaderMotor.set(0);
  }

  @Override
  public void periodic() {
    // Continuously push telemetry data to SmartDashboard for tuning and debugging
    SmartDashboard.putNumber("Flywheel/Velocity", getFlywheelVelocity());
    SmartDashboard.putNumber("Flywheel/Leader Current", flywheelLeaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/Follower Current", flywheelFollowerMotor.getOutputCurrent());
  }
}
