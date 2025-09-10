package frc.robot.util;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants;
//import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants;
//import frc.robot.subsystems.shooter.shooterwrist.ShooterWristConstants;

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

  public static double intakeRollerStateToVelocity(IntakeRollerConstants.IntakeRollerStates state) {
    return IntakeRollerConstants.intakeRollerVelocities[state.getIndex()];
  }

  //public static double shooterRollerStateToVelocity(ShooterRollerConstants.ShooterRollerStates state) {
    //return ShooterRollerConstants.shooterRollerVelocities[state.getIndex()];
  //}
   
   /*
  public static double rollersStateToVelocity(IntakeRollersConstants.RollersStates state) {
    return IntakeRollersConstants.rollersVelocities[state.getIndex()];
  }

  public static double shooterRollersStateToVelocity(ShooterRollersConstants.ShooterRollersStates state) {
    return ShooterRollersConstants.rollersVelocities[state.getIndex()];
  }
  public static double climbStateToHeight(ClimbConstants.ClimbStates state) {
    return (ClimbConstants.setpoint[state.getIndex()]);
  }
  
  
  public static double shooterWristStateToSetpoint(WristStates state) {
    return (ShooterWristConstants.setpoints[state.getIndex()]);
    }
    */
    public static double intakeWristStateToSetpoint(IntakeWristConstants.intakeWristStates state) {
      return (IntakeWristConstants.intakeWristSetpoints[state.getIndex()]);
    }

    public static double shooterWristStateToSetpoint(ShooterWristConstants.ShooterWristStates state) {
      return (ShooterWristConstants.shooterWristSetpoints[state.getIndex()]);
    }
    //public static double shooterWristStateToSetpoint(ShooterWristConstants.ShooterWristStates state) {
      //return (ShooterWristConstants.shooterWristSetpoints[state.getIndex()]);
    //}
}