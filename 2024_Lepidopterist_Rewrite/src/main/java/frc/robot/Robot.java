// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollers;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollersIOTalonFX;
import frc.robot.subsystems.ramprollers.RampRollers;
import frc.robot.subsystems.ramprollers.RampRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndeffectorRollers endeffectorRollers;

  private final Rollers rollers;

  private final CommandXboxController controller = new CommandXboxController(0);

  private Command m_autonomousCommand;

  public Robot() {
    rampRollers = new RampRollers(new RampRollersIOTalonFX());
    endeffectorRollers = new EndeffectorRollers(new EndeffectorRollersIOTalonFX());
    rollers = new Rollers(endeffectorRollers, rampRollers);

    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    rampRollers.updateInputs();
    endeffectorRollers.updateInputs();
    rollers.updateInputs();
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
    controller.b().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.INTAKING)));

    controller.rightBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ROLLERS_INTAKE_CORAL)));

    controller.rightBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_INTAKE_ALGAE)));

    controller.rightTrigger().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_SCORE_CORAL)));

    controller.rightTrigger().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_SCORE_ALGAE)));

    controller.x().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.STOPPED)));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
