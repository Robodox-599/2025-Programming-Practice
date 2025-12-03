package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.endefectorrollerS.EndefectorRollers;
import frc.robot.subsystems.endefectorrollerS.EndefectorRollersIOTalonFX;
import frc.robot.subsystems.ramprollerS.RampRollers;
import frc.robot.subsystems.ramprollerS.RampRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndefectorRollers endefectorRollers;
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
    endefectorRollers = new EndefectorRollers(new EndefectorRollersIOTalonFX());
    configureBindings();
  }
  @Override
  public void robotPeriodic() {
    rampRollers.updateInputs();
    endefectorRollers.updateInputs();
    CommandScheduler.getInstance().run();
    // endefectorRollers.updateInputs();
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
    //Sets the state to intaking for both ramp & endefector rollers
    controller.rightBumper().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.INTAKING)));
    controller.leftBumper().onTrue(Commands.runOnce(() -> endefectorRollers.setWantedState(EndefectorRollers.WantedState.INTAKING)));

    //Stops both ramp & endefector rollers
    controller.x().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.STOPPED)));
    controller.y().onTrue(Commands.runOnce(() -> endefectorRollers.setWantedState(EndefectorRollers.WantedState.STOPPED)));

    //Sets the state to score for both ramp & endefector rollers
    controller.rightTrigger().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.SCORE)));
    controller.leftTrigger().onTrue(Commands.runOnce(() -> endefectorRollers.setWantedState(EndefectorRollers.WantedState.SCORE)));
  }
  
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
  }

