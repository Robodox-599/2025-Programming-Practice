package frc.robot.util;

import frc.robot.subsystems.indexer.IndexerConstants;

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

  public static double rollersStateToVelocity(IndexerConstants.IndexerStates state) {
    return IndexerConstants.indexerVelocities[state.getIndex()];
  }
  /*
  public static double climbStateToHeight(ClimbConstants.ClimbStates state) {
    return (ClimbConstants.setpoint[state.getIndex()]);
  }

  public static double wristStateToSetpoint(WristConstants.WristStates state) {
    return (WristConstants.setpoints[state.getIndex()]);
  }
     */
}