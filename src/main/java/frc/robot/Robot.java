package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer for button bindings and autonomous choosers.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after robot disable.
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    // Runs the CommandScheduler. Must be called every loop for command-based to function.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (m_robotContainer != null) {
      m_robotContainer.setMotorBrake(true);
    }
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      if (m_robotContainer != null) {
        m_robotContainer.setMotorBrake(false);
      }
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    if (m_robotContainer != null) {
      m_robotContainer.setMotorBrake(true);
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    }

    System.out.println("Auto selected: " + (m_autonomousCommand != null ? m_autonomousCommand.getName() : "None"));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
