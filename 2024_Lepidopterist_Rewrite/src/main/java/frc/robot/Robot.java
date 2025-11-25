package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.RampRollers.RampRollers;
import frc.robot.Subsystems.RampRollers.RampRollers.WantedState;
import frc.robot.Subsystems.RampRollers.RampRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final CommandXboxController controller = new CommandXboxController(0);

  // private Command m_autonomousCommand;

  public Robot(){

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    rampRollers = new RampRollers(new RampRollersIOTalonFX());
    configureBindings();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    // if (m_autonomousCommand != null) {
      // m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  @Override
  public void teleopPeriodic() {
    rampRollers.updateInputs();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    controller.rightBumper().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(WantedState.INTAKING)));

    controller.x().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(WantedState.STOPPED)));

    controller.rightTrigger().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(WantedState.SCORE)));

  }
  
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
  }

