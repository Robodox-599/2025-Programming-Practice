package frc.robot;

import dev.doglog.DogLog;

public class SafetyChecker {
  private double shooterVelocity;
  private double indexerVelocity;
  private boolean hasGamePiece = false;
  private boolean isIntakeRunning = false;
  private boolean isShooterRunning = false;
  private boolean isIndexerRunning = false;
  
  // Safety thresholds
  private final double minimumShooterVelocityForFeed = 0.35; // minimum shooter velocity before feeding game piece
  private final double maxIndexerVelocityWithoutShooter = 0.1; // max indexer speed when shooter isn't ready
  
  // Setpoint tracking
  private boolean isAtSetpointShooter = false;
  private boolean isAtSetpointIndexer = false;
  private boolean isAtSetpointIntake = false;

  // Set current shooter velocity - should be called every loop cycle
  public void setCurrentShooterVelocity(double shooterVelocity) {
    this.shooterVelocity = shooterVelocity;
    DogLog.log("SafetyChecker/CurrentShooterVelocity", this.shooterVelocity);
  }

  // Set current indexer velocity - should be called every loop cycle
  public void setCurrentIndexerVelocity(double indexerVelocity) {
    this.indexerVelocity = indexerVelocity;
    DogLog.log("SafetyChecker/CurrentIndexerVelocity", this.indexerVelocity);
  }

  // Update whether we have a game piece in the robot
  public void setHasGamePiece(boolean hasGamePiece) {
    this.hasGamePiece = hasGamePiece;
    DogLog.log("SafetyChecker/HasGamePiece", this.hasGamePiece);
  }

  // Update subsystem running states
  public void setIntakeRunning(boolean isRunning) {
    this.isIntakeRunning = isRunning;
    DogLog.log("SafetyChecker/IntakeRunning", this.isIntakeRunning);
  }

  public void setShooterRunning(boolean isRunning) {
    this.isShooterRunning = isRunning;
    DogLog.log("SafetyChecker/ShooterRunning", this.isShooterRunning);
  }

  public void setIndexerRunning(boolean isRunning) {
    this.isIndexerRunning = isRunning;
    DogLog.log("SafetyChecker/IndexerRunning", this.isIndexerRunning);
  }

  // Update setpoint status for each subsystem
  public void updateIsAtSetpointShooter(boolean value) {
    isAtSetpointShooter = value;
    DogLog.log("SafetyChecker/AtSetpointShooter", isAtSetpointShooter);
  }

  public void updateIsAtSetpointIndexer(boolean value) {
    isAtSetpointIndexer = value;
    DogLog.log("SafetyChecker/AtSetpointIndexer", isAtSetpointIndexer);
  }

  public void updateIsAtSetpointIntake(boolean value) {
    isAtSetpointIntake = value;
    DogLog.log("SafetyChecker/AtSetpointIntake", isAtSetpointIntake);
  }

  // Check if it's safe to run the indexer at full speed
  // Prevents feeding game pieces when shooter isn't ready
  public boolean isSafeIndexer() {
    boolean safe = !hasGamePiece || isShooterAtSpeed() || Math.abs(indexerVelocity) <= maxIndexerVelocityWithoutShooter;
    DogLog.log("SafetyChecker/isSafeIndexer", safe);
    return safe;
  }

  // Check if it's safe to run the shooter
  // Always safe unless there are specific conditions you want to prevent
  public boolean isSafeShooter() {
    // Add any shooter-specific safety checks here
    // For example, don't run shooter if intake is jammed, etc.
    boolean safe = true;
    DogLog.log("SafetyChecker/isSafeShooter", safe);
    return safe;
  }

  // Check if it's safe to run the intake
  // Prevents intake when shooter is running at high speed (if needed)
  public boolean isSafeIntake() {
    // You might want to prevent intake when shooting
    boolean safe = !isShootingAtHighSpeed();
    DogLog.log("SafetyChecker/isSafeIntake", safe);
    return safe;
  }

  // Check if shooter is at target speed for shooting
  public boolean isShooterAtSpeed() {
    boolean atSpeed = Math.abs(shooterVelocity) >= minimumShooterVelocityForFeed;
    DogLog.log("SafetyChecker/ShooterAtSpeed", atSpeed);
    return atSpeed;
  }

  // Check if shooter is running at high speed that might interfere with intake
  public boolean isShootingAtHighSpeed() {
    return Math.abs(shooterVelocity) > minimumShooterVelocityForFeed;
  }

  // Check if it's safe to feed game piece to shooter
  // Ensures shooter is at proper speed before feeding
  public boolean isSafeToFeed() {
    boolean safe = hasGamePiece && isShooterAtSpeed() && isAtSetpointShooter;
    DogLog.log("SafetyChecker/SafeToFeed", safe);
    return safe;
  }

   // Check if robot is ready to shoot
   // All systems must be at setpoint and conditions met
  public boolean isReadyToShoot() {
    boolean ready = isAtSetpointShooter && hasGamePiece && isShooterAtSpeed();
    DogLog.log("SafetyChecker/ReadyToShoot", ready);
    return ready;
  }

   // Check if robot is ready to intake
   // Ensures no conflicts with other systems
  public boolean isReadyToIntake() {
    boolean ready = !hasGamePiece && isSafeIntake() && isAtSetpointIntake;
    DogLog.log("SafetyChecker/ReadyToIntake", ready);
    return ready;
  }

  // Get overall system safety status
  public boolean isSystemSafe() {
    boolean safe = isSafeShooter() && isSafeIndexer() && isSafeIntake();
    DogLog.log("SafetyChecker/SystemSafe", safe);
    return safe;
  }

  // Getters for setpoint status
  public boolean isAtSetpointShooter() {
    return isAtSetpointShooter;
  }

  public boolean isAtSetpointIndexer() {
    return isAtSetpointIndexer;
  }

  public boolean isAtSetpointIntake() {
    return isAtSetpointIntake;
  }

  // Getters for current states
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  public double getCurrentShooterVelocity() {
    return shooterVelocity;
  }

  public double getCurrentIndexerVelocity() {
    return indexerVelocity;
  }
}