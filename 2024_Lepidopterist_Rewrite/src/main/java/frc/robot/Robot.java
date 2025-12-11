// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.endEffectorRollers.EndEffectorRollersIOTalonFX;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.endEffectorRollers.EndEffectorRollers;
import frc.robot.subsystems.rampRollers.RampRollers;
import frc.robot.subsystems.rampRollers.RampRollersIOTalonFX;



public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndEffectorRollers endEffectorRollers;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Rollers rollers;


  private Command m_autonomousCommand;

  //first thing that runs when program starts
  public Robot() {
    // if in real mode
    rampRollers = new RampRollers(new RampRollersIOTalonFX());
    endEffectorRollers = new EndEffectorRollers(new EndEffectorRollersIOTalonFX());
    rollers = new Rollers(rampRollers, endEffectorRollers);

    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    rampRollers.updateInputs();
    endEffectorRollers.updateInputs();
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

    // meer stinks
      
  
    private void configureBindings() {
      // b intakeing coral from ramp only
      // right bumper intake coral from both
      //right trigger is to score coral/

      // left bumper is to intake algae
      // left trigger is to score algae
      // stopp should be X
      controller.rightBumper().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedSuperState.ROLLERS_INTAKING)));
      controller.rightTrigger().onTrue(Commands.runOnce(() -> endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING)));
      controller.leftBumper().onTrue(Commands.runOnce(() -> endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING)));
      controller.leftTrigger().onTrue(Commands.runOnce (() -> endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING)));
      controller.x().onTrue(Commands.runOnce(() -> rollers.setWantedState(Rollers.WantedSuperState.STOPPED)).alongWith(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.STOPPED))));
      controller.b().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.INTAKING)));
    }
  
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
}