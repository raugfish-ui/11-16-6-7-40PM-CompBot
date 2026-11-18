package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex climberLeaderMotor;
  private final SparkFlex climberFollowerMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberClosedLoopController;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberLeaderMotor = new SparkFlex(CLIMBER_LEADER_ID, MotorType.kBrushless);
    climberFollowerMotor = new SparkFlex(CLIMBER_FOLLOWER_ID, MotorType.kBrushless);

    climberClosedLoopController = climberLeaderMotor.getClosedLoopController();
    climberEncoder = climberLeaderMotor.getEncoder();

    // Configure Leader Motor
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig.inverted(true);
    leaderConfig.smartCurrentLimit(CLIMBER_CURRENT_LIMIT);

    leaderConfig.encoder.positionConversionFactor(1.0);
    leaderConfig.encoder.velocityConversionFactor(1.0);
    
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(CLIMBER_MOTOR_P, ClosedLoopSlot.kSlot0)
        .i(CLIMBER_MOTOR_I, ClosedLoopSlot.kSlot0)
        .d(CLIMBER_MOTOR_D, ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

    leaderConfig.softLimit
        .forwardSoftLimit(CLIMBER_MOTOR_FWD_LIMIT)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(CLIMBER_MOTOR_REV_LIMIT)
        .reverseSoftLimitEnabled(false);

    climberLeaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure Follower Motor
    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.inverted(false);
    followerConfig.smartCurrentLimit(CLIMBER_CURRENT_LIMIT);
    followerConfig.follow(CLIMBER_LEADER_ID);
    
    climberFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize dashboard tune target
    SmartDashboard.putNumber("Climber Voltage Target", CLIMBER_VOLTAGE);
  }

  /** Sets the voltage of the climber motors. */
  public void setClimberVoltage(double voltage) {
    climberLeaderMotor.setVoltage(voltage);
  }

  /** Runs the climber using closed-loop velocity control. */
  public void setClimberVelocity(double velocity) {
    climberClosedLoopController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  /** Gets the current velocity of the climber. */
  public double getClimberVelocity() {
    return climberEncoder.getVelocity();
  }

  /** Gets the current position of the climber. */
  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  /** Stops the climber motors. */
  public void stop() {
    climberLeaderMotor.set(0);
  }

  @Override
  public void periodic() {
    // Continuously publish telemetry data for debugging and dashboard tuning
    SmartDashboard.putNumber("Climber/Velocity", getClimberVelocity());
    SmartDashboard.putNumber("Climber/Position", getClimberPosition());
    SmartDashboard.putNumber("Climber/Leader Current", climberLeaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/Follower Current", climberFollowerMotor.getOutputCurrent());
  }
}
