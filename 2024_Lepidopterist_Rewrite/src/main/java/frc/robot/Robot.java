// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollers;
import frc.robot.subsystems.endeffectorrollers.EndeffectorRollersIOTalonFX;
import frc.robot.subsystems.endeffectorwrist.EndeffectorWrist;
import frc.robot.subsystems.endeffectorwrist.EndeffectorWristIOTalonFX;
import frc.robot.subsystems.ramprollers.RampRollers;
import frc.robot.subsystems.ramprollers.RampRollersIOTalonFX;

public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndeffectorRollers endeffectorRollers;
  private final EndeffectorWrist endeffectorWrist;

  private final SuperStructure superStructure;

  private final CommandXboxController controller = new CommandXboxController(0);

  private Command m_autonomousCommand;

  public Robot() {
    rampRollers = new RampRollers(new RampRollersIOTalonFX());
    endeffectorRollers = new EndeffectorRollers(new EndeffectorRollersIOTalonFX());
    endeffectorWrist = new EndeffectorWrist(new EndeffectorWristIOTalonFX());
    superStructure = new SuperStructure(endeffectorRollers, rampRollers, endeffectorWrist);

    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    superStructure.updateInputs();
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
    // controller.y().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.INTAKING)));

    // controller.rightBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ROLLERS_INTAKE_CORAL)));

    // controller.leftBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_INTAKE_ALGAE)));

    // controller.rightTrigger().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_SCORE_CORAL)));

    // controller.leftTrigger().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedState.ENDEFFECTOR_SCORE_ALGAE)));

    controller.rightBumper().onTrue(Commands.runOnce(() -> endeffectorWrist.setWantedState(EndeffectorWrist.WantedState.PREPARED)));

    controller.leftBumper().onTrue(Commands.runOnce(() -> endeffectorWrist.setWantedState(EndeffectorWrist.WantedState.INTAKING_ALGAE_REEF)));

    controller.rightTrigger().onTrue(Commands.runOnce(() -> endeffectorWrist.setWantedState(EndeffectorWrist.WantedState.SCORING_CORAL)));

    controller.x().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.STOPPED)).alongWith(Commands.runOnce(() -> superStructure.setWantedState(SuperStructure.WantedState.STOPPED))));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
