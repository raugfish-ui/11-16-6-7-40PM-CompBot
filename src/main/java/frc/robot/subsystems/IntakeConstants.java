package frc.robot.subsystems;

public final class IntakeConstants {
    // CAN IDs (ensure these match your physical wiring and don't conflict with swerve)
    public static final int kPivotMotorID = 15;
    public static final int kRollerMotorID = 16;

    // Current limits for NEO / brushless motors (in Amps)
    public static final int kPivotCurrentLimit = 30;
    public static final int kRollerCurrentLimit = 40;

    // Default motor speeds for intake operations
    public static final double kIntakeRollerSpeed = 0.8;    // Speed to pull game pieces in
    public static final double kOuttakeRollerSpeed = -0.6;  // Speed to push game pieces out
    public static final double kPivotMoveSpeed = 0.3;       // Speed for moving the pivot manually/safely
}
