package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotorHelper {
  
  /** Helper to quickly create a brushless SPARK MAX */
  public static SparkMax createSparkMax(int channel) {
    return new SparkMax(channel, MotorType.kBrushless);
  }

  /** Helper to quickly create a Falcon / TalonFX motor */
  public static TalonFX createTalonFX(int channel) {
    TalonFX motor = new TalonFX(channel);
    motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    return motor;
  }
}
