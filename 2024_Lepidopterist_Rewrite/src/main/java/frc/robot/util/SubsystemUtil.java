package frc.robot.util;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.intakerollers.IntakeRollersConstants;
import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants;
import frc.robot.subsystems.intake.intakewrist.IntakeWristConstants.WristStates;
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollersConstants;
import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants;

public class SubsystemUtil {
  /*
  public static double convertToTicks(double height) {
    return height / ElevatorConstants.inchesPerRev;
  }

  public static double elevatorStateToHeightTicks(ElevatorConstants.ElevatorStates state) {
    return convertToTicks(ElevatorConstants.heights[state.getIndex()]);
  }

  public static double elevatorStateToHeightInches(ElevatorConstants.ElevatorStates state) {
    return ElevatorConstants.heights[state.getIndex()];
  } */

  public static double indexerStateToVelocity(IndexerConstants.IndexerStates state) {
    return IndexerConstants.indexerVelocities[state.getIndex()];
  }

  public static double rollersStateToVelocity(IntakeRollersConstants.RollersStates state) {
    return IntakeRollersConstants.rollersVelocities[state.getIndex()];
  }

  public static double shooterRollersStateToVelocity(ShooterRollersConstants.ShooterRollersStates state) {
    return ShooterRollersConstants.rollersVelocities[state.getIndex()];
  }
  /*
  public static double climbStateToHeight(ClimbConstants.ClimbStates state) {
    return (ClimbConstants.setpoint[state.getIndex()]);
  }
  */

  public static double wristStateToSetpoint(IntakeWristConstants.WristStates state) {
    return (IntakeWristConstants.setpoints[state.getIndex()]);
  }

  public static double shooterWristStateToSetpoint(WristStates state) {
    return (ShooterWristConstants.setpoints[state.getIndex()]);
  }
}