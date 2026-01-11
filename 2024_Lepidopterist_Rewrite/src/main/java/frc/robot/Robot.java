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
import frc.robot.subsystems.endEffectorWrist.EndEffectorWrist;
import frc.robot.subsystems.endEffectorWrist.EndEffectorWristIOTalonFX;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.endEffectorRollers.EndEffectorRollers;
import frc.robot.subsystems.rampRollers.RampRollers;
import frc.robot.subsystems.rampRollers.RampRollersIOTalonFX;



public class Robot extends TimedRobot {
  private final RampRollers rampRollers;
  private final EndEffectorRollers endEffectorRollers;
  private final EndEffectorWrist endEffectorWrist;
  private final CommandXboxController controller = new CommandXboxController(0);
  private final SuperStructure superStructure;

  private Command m_autonomousCommand;

  //first thing that runs when program starts
  public Robot() {
    // if in real mode
    rampRollers = new RampRollers(new RampRollersIOTalonFX());
    endEffectorRollers = new EndEffectorRollers(new EndEffectorRollersIOTalonFX());
    endEffectorWrist = new EndEffectorWrist(new EndEffectorWristIOTalonFX());
    
    superStructure = new SuperStructure(rampRollers, endEffectorRollers, endEffectorWrist);

    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    rampRollers.updateInputs();
    endEffectorRollers.updateInputs();
    superStructure.updateInputs();
    endEffectorWrist.updateInputs();

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
      // right trigger is to score coral/
      // left bumper is to intake algae
      // left trigger is to score algae
      // stop should be X
      
      //ramp roller intaking coral
      controller.rightBumper().onTrue(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.INTAKING)));
      //scores coral
      controller.rightTrigger().onTrue(Commands.runOnce(() -> superStructure.setWantedState(SuperStructure.WantedSuperState.SCORING_CORAL)));
      //intakes algae
      controller.leftBumper().onTrue(Commands.runOnce(() -> superStructure.setWantedState(SuperStructure.WantedSuperState.INTAKING_ALGAE)));
      //scores algae
      controller.leftTrigger().onTrue(Commands.runOnce (() -> superStructure.setWantedState(SuperStructure.WantedSuperState.SCORING_ALGAE)));
      //stops robot
      controller.x().onTrue(Commands.runOnce(() -> superStructure.setWantedState(SuperStructure.WantedSuperState.STOPPED)).alongWith(Commands.runOnce(() -> rampRollers.setWantedState(RampRollers.WantedState.STOPPED))));
      //wrist goes to position handing_coral
      controller.y().onTrue(Commands.runOnce(() -> endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.HANDING_CORAL)));
      //wrist goes to position prepared
      controller.b().onTrue(Commands.runOnce(() -> endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.PREPARED)));
      //wrist goes to position scoring_coral
      controller.a().onTrue(Commands.runOnce(() -> endEffectorWrist.setWantedState(EndEffectorWrist.WantedState.SCORING_CORAL)));

    }
  
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
}