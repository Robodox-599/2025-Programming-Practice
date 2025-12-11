// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.subsystems.endEffectorRollers.EndEffectorRollers;
import frc.robot.subsystems.endEffectorRollers.EndEffectorRollersIO;
import frc.robot.subsystems.rampRollers.RampRollers;

/** Add your docs here. */
public class Rollers {
    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private final EndEffectorRollers endEffectorRollers;
    private final RampRollers rampRollers;


    //used to acces rampRollers & Endeffector rollers to call later
    public Rollers(RampRollers rampRollers, EndEffectorRollers endEffectorRollers){
        // 'rampRollers' (not 'this') is the RampRollers object passed into this constructor (the special setup function that runs when you create an object, in this case public Rollers(){}"").
        // "RampRollers & EndEffectorRollers" is the object type. Just like double. However, the object type is the name of the method/Blueprint which is "RampRollers.java"
        // To add one, a constructur is a special type of method, that is only used to build objects
        // 'this.rampRollers' is the variable that lives inside this Rollers object.
        // We store the passed-in rampRollers inside this object using 'this.' so this Rollers class can use it later.
        // really useful tip: by clicking on the object, of either "this"'s' object or the obejct it self, highlights exactly what it's refering to the code

        //the constructer which is public Rollers(){} is saying: “I need a RampRollers object type and EndEffectorRollers type to work with.”
        // basically the constructur allows the creation of objects from classes

        this.rampRollers = rampRollers; //setting up the fields, the field is "this.rampRollers" = to an object so we can easily call later
        this.endEffectorRollers = endEffectorRollers;
    }

    public enum WantedSuperState {
      //isolate rollers_Holding for holding 2 game pieces
        ROLLERS_INTAKING_CORAL,
        ENDEFFECTOR_HOLDING_CORAL,
        ENDEFFECTOR_SCORING_CORAL,
        ENDEFFECTOR_INTAKING_ALGAE,
        ENDEFFECTOR_HOLDING_ALGAE,
        ENDEFFECTOR_SCORING_ALGAE,
        STOPPED
    }


    public enum CurrentSuperState {
        ROLLERS_INTAKING_CORAL,
        ENDEFFECTOR_HOLDING_CORAL,
        ENDEFFECTOR_SCORING_CORAL,
        ENDEFFECTOR_INTAKING_ALGAE,
        ENDEFFECTOR_HOLDING_ALGAE,
        ENDEFFECTOR_SCORING_ALGAE,
        STOPPED
    }


    // Specifically the order for methods of updateInputs, handleStateTransitions, & applyStates matters, this keeps organization clear by the general order of the logic.
    public void updateInputs(){
        handleStateTransitions();
        applyStates();
    }

    //based on the wanted state it determines what the current/wanted state the SUBSYSTEM is in
    private void handleStateTransitions(){
        switch(wantedSuperState){
            case ROLLERS_INTAKING_CORAL:
            if (endEffectorRollers.isCoralDetected() && !rampRollers.isCoralDetected()){ // detect coral in ee but not in ramp, which means we should HOLD THE CORAL
                // function to set wantedCoralPosition to the current position of the MOTOR
                endEffectorRollers.setEndeffectorHoldCoralPosition();
                wantedSuperState = WantedSuperState.ENDEFFECTOR_HOLDING_CORAL;
                currentSuperState = CurrentSuperState.ENDEFFECTOR_HOLDING_CORAL;
              } else {
                currentSuperState = CurrentSuperState.ROLLERS_INTAKING_CORAL;
              }
              break;
            case ENDEFFECTOR_HOLDING_CORAL:
              if (!endEffectorRollers.isCoralDetected()){
                wantedSuperState = WantedSuperState.STOPPED;                
                currentSuperState = CurrentSuperState.STOPPED;
              }  else{
                currentSuperState = CurrentSuperState.ENDEFFECTOR_HOLDING_CORAL;
              }
              break;
            case ENDEFFECTOR_SCORING_CORAL:
              if (!endEffectorRollers.isCoralDetected()){
                wantedSuperState = WantedSuperState.STOPPED;
                currentSuperState = CurrentSuperState.STOPPED;
              } else {
                currentSuperState = CurrentSuperState.ENDEFFECTOR_SCORING_CORAL;
              }
              break;
            case ENDEFFECTOR_INTAKING_ALGAE:
              if(endEffectorRollers.isAlgaeDetected()){
                wantedSuperState = WantedSuperState.ENDEFFECTOR_HOLDING_ALGAE;
                currentSuperState = CurrentSuperState.ENDEFFECTOR_HOLDING_CORAL;
              }
              currentSuperState = CurrentSuperState.ENDEFFECTOR_INTAKING_ALGAE;
              break;  
            case ENDEFFECTOR_HOLDING_ALGAE:
              if(!endEffectorRollers.isAlgaeDetected()){
                wantedSuperState= WantedSuperState.STOPPED;
                currentSuperState = CurrentSuperState.STOPPED;
              } else{
                currentSuperState = CurrentSuperState.ENDEFFECTOR_HOLDING_ALGAE;
              }
              break;
            case ENDEFFECTOR_SCORING_ALGAE:
              if(!endEffectorRollers.isAlgaeDetected()){
                wantedSuperState = WantedSuperState.STOPPED;
                currentSuperState = CurrentSuperState.STOPPED;
              } else{
                currentSuperState = CurrentSuperState.ENDEFFECTOR_SCORING_ALGAE;
              }
              break;
            case STOPPED:
                currentSuperState = CurrentSuperState.STOPPED;              
              break;
            default:
              currentSuperState = CurrentSuperState.STOPPED;
              break;
        }
    }

    //based on the current state the SUBSYSTEM is in, it does sometype of action
    private void applyStates(){
        switch(currentSuperState){
            case STOPPED:
                // Using the dot operator to access the object's function
                endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.STOPPED);
                break;
            case ROLLERS_INTAKING_CORAL:
              rampRollers.setWantedState(RampRollers.WantedState.TRANSFERING);                
              // We are allowed to use the setWantedState() method although it's in another class because we imported the class "EndEffectorRollers.Java" here which allows us to use function(method) meant for the EndEffector here
              // click the setWantedState() function or any function to see where it originates from
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING_CORAL);
                break;
            case ENDEFFECTOR_HOLDING_CORAL:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.HOLDING_CORAL);
              break;
            case ENDEFFECTOR_SCORING_CORAL:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING_CORAL);
              break;
            case ENDEFFECTOR_INTAKING_ALGAE:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.INTAKING_ALGAE);
              break;
            case ENDEFFECTOR_HOLDING_ALGAE:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.HOLDING_ALGAE);
              break;
            case ENDEFFECTOR_SCORING_ALGAE:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.SCORING_ALGAE);
              break;
            default:
              endEffectorRollers.setWantedState(EndEffectorRollers.WantedState.STOPPED);
                break;

            
        }
    }

    public void setWantedState(WantedSuperState wantedSuperState){
      this.wantedSuperState = wantedSuperState;
    }

}
