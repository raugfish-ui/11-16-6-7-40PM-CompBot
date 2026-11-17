package frc.robot.commands;

public final class SubsystemConstants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class IntakeConstants {
    // CAN IDs for your Spark Max motor controllers (adjust these to match your physical robot wiring)
    public static final int kRollerMotorID = 10;
    public static final int kPivotMotorID = 11;

    // Default motor speeds for intaking, reversing, and pivoting
    public static final double kRollerInSpeed = 0.8;
    public static final double kRollerOutSpeed = -0.8;
    public static final double kPivotRaiseSpeed = 0.3;
    public static final double kPivotLowerSpeed = -0.3;
  }
}
