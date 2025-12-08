// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.rollers.RampRollers.RampRollersIO;
import frc.robot.Subsystems.rollers.endeffectorrollers.EndeffectorRollersIO;

public class RollersStateMachine extends SubsystemBase {
  
    private final EndeffectorRollersIO endIO;
    private final RampRollersIO rampIO;

  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    RAMP_INTAKING,
    RAMP_HOLD_CORAL,
    ROLLERS_INTAKE,
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_SCORE,
    STOPPED,
  }

  public enum CurrentState{
    RAMP_INTAKING,
    RAMP_HOLD_CORAL,
    ROLLERS_INTAKE,
    ENDEFFECTOR_HOLD_CORAL,
    ENDEFFECTOR_SCORE,
    STOPPED,
  }

  public void handleStateTransitions(){
    
  }

  public RollersStateMachine(EndeffectorRollersIO endIO, RampRollersIO rampIO) {
    this.endIO = endIO;
    this.rampIO = rampIO;
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }
}
