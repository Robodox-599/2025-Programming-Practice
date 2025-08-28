// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakewrist;

public class IntakeWristConstants {

  // wrist states (not 100% sure these are all of the needed states)
  public static enum IntakeWristStates {
    INTAKING(0),
    STOW(1),
    STOP(2);

    private final int index;

    IntakeWristStates(int index) {
      this.index = index;
    }

    public int getIndex() {
      return index;
    }
  }

  // Setpoints

  public static final double[] intakeWristSetpoints = {
    0.6, // INTAKING
    0.4, // STOW
    0.0 // STOP
  };

  // Real motor config constants
  public static final int wristMotorID = 13;
  public static final String wristMotorCANBus = "rio";
  public static final double gearRatio = 0;
  public static final double wristMOI = 0.04;
  public static final double wristPositionTolerance = 0;
  public static final boolean EnableCurrentLimit = true;
  public static final int ContinousCurrentLimit = 50;
  public static final int PeakCurrentLimit = 50;
  public static final double PeakCurrentDuration = 0.1;
  public static final double maxWristVelocity = 67.0;
  public static final double maxWristAccel = 67.0;

  // PID consants

  public static final double realkP = 0.0;
  public static final double realkI = 0.0;
  public static final double realkD = 0.0;
  public static final double realkS = 0.0;
  public static final double realkV = 0.0;
  public static final double realkG = 0.0;

  // Sim PID constants

  public static final double simkP = 6.9;
  public static final double simkI = 0.5;
  public static final double simkD = 2.25;
  public static final double simkV = 0.0;
  public static final double simkS = 0.0;
  public static final double simVelocityConstant = 0.2;

  // Wrist angle clamps

  public static final double wristMinAngle = 0.0;
  public static final double wristMaxAngle = 0.0;
}