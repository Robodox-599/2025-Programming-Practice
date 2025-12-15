// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffectorWrist;

//the state machine
public class EndEffectorWrist extends EndEffectorWristIO {
    
    public enum WantedState{
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_GROUND_ALGAE,
        INTAKING_REEF_ALGAE,
        SCORING_NET_ALGAE,
        SCORING_PROCESSOR_ALGAE
    }

    public enum CurrentState{
        PREPARED,
        HANDING_CORAL,
        SCORING_CORAL,
        INTAKING_GROUND_ALGAE,
        INTAKING_REEF_ALGAE,
        SCORING_NET_ALGAE,
        SCORING_PROCESSOR_ALGAE
    }
}
