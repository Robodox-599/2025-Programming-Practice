package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.endefectorrollers.EndefectorRollers;
import frc.robot.subsystems.endefectorrollers.EndefectorRollersIOTalonFX;
import frc.robot.subsystems.ramprollers.RampRollers;
import frc.robot.subsystems.ramprollers.RampRollersIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;


public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndefectorRollers endefectorRollers;
  private final Rollers rollers;
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
    rollers = new Rollers(endefectorRollers, rampRollers);
    configureBindings();
  }
  @Override
  public void robotPeriodic() {
    rampRollers.updateInputs();
    endefectorRollers.updateInputs();
    rollers.updateInputs();
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
    controller.rightBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.wantedSuperState.ROLLERS_INTAKING)));
    controller.leftBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.wantedSuperState.RAMP_INTAKING)));

    //Stops both ramp & endefector rollers
    controller.x().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.wantedSuperState.STOPPED)));

    //Sets the state to score for both ramp & endefector rollers
    controller.rightTrigger().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.wantedSuperState.ENDEFECTOR_SCORE)));
  }
  
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
  }

