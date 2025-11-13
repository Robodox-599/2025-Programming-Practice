// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.RampRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampRollers {
  private final RampRollersIO io;


  /** Creates a new RampRollers. */
  public RampRollers(RampRollersIO io){
    //this calls the one from the class (global variable)
    //clicking on it shows which one ur refering to
    this.io = io;


  }

  public void updateInputs() {
    
  }
}
