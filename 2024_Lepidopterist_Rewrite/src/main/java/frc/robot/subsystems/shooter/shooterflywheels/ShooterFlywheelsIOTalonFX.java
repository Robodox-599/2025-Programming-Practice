package frc.robot.subsystems.shooter.shooterflywheels;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants;
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;
import frc.robot.subsystems.shooter.shooterflywheels.ShooterFlywheelsConstants.ShooterFlywheelsStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ShooterFlywheelsIOTalonFX extends ShooterFlywheelsIO {
  private final TalonFX topFlywheelMotor;
  private final TalonFX bottomFlywheelMotor;
  TalonFXConfiguration indexerConfig;
  Debouncer beamBreakDebouncer = new Debouncer(beamBreakDebounce);
  private Timer beamBreakTimer = new Timer(); 
  private DigitalInput beamBreak;
  private ShooterFlywheelsConstants.ShooterFlywheelsStates currentState = ShooterFlywheelsStates.STOP;

  private final StatusSignal<AngularVelocity> topVelocity;
  private final StatusSignal<Voltage> topAppliedVolts;
  private final StatusSignal<Current> topStatorCurrent;
  private final StatusSignal<Current> topSupplyCurrent;
  private final StatusSignal<Temperature> topTemperature;

  private final StatusSignal<AngularVelocity> bottomVelocity;
  private final StatusSignal<Voltage> bottomAppliedVolts;
  private final StatusSignal<Current> bottomStatorCurrent;
  private final StatusSignal<Current> bottomSupplyCurrent;
  private final StatusSignal<Temperature> bottomTemperature;

  private double desiredVelocity;

  public ShooterFlywheelsIOTalonFX() {
    topFlywheelMotor = new TalonFX(topFlywheelMotorID, topMotorCANBus);
    bottomFlywheelMotor = new TalonFX(bottomFlywheelMotorID, bottomMotorCANBus);
    beamBreak = new DigitalInput(ShooterFlywheelsConstants.beamBreakPort);
    beamBreakTimer.start();

    topMotorConfig = new TalonFXConfiguration();
    bottomMotorConfig = new TalonFXConfiguration();

    topFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);

    // top motor
    topMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    topMotorConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    topMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    topMotorConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    topMotorConfig.Slot0.kP = realP;
    topMotorConfig.Slot0.kI = realI;
    topMotorConfig.Slot0.kD = realD;
    topMotorConfig.Slot0.kS = realS;
    topMotorConfig.Slot0.kV = realV;

    // bottom motor
    bottomMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    bottomMotorConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    bottomMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    bottomMotorConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    bottomMotorConfig.Slot0.kP = realP;
    bottomMotorConfig.Slot0.kI = realI;
    bottomMotorConfig.Slot0.kD = realD;
    bottomMotorConfig.Slot0.kS = realS;
    bottomMotorConfig.Slot0.kV = realV;

    beamBreakDebouncer.setDebounceType(DebounceType.kFalling);

    PhoenixUtil.tryUntilOk(10, () -> topFlywheelMotor.getConfigurator().apply(topMotorConfig, 1));
    intakeRollerMotor.optimizeBusUtilization();

    PhoenixUtil.tryUntilOk(10, () -> bottomFlywheelMotor.getConfigurator().apply(bottomMotorConfig, 1));
    bottomFlywheelMotor.optimizeBusUtilization();

    topVelocity = topFlywheelMotor.getVelocity();
    topAppliedVolts = topFlywheelMotor.getMotorVoltage();
    topStatorCurrent = topFlywheelMotor.getStatorCurrent();
    topTemperature = topFlywheelMotor.getDeviceTemp();
    topSupplyCurrent = topFlywheelMotor.getSupplyCurrent();

    bottomVelocity = bottomFlywheelMotor.getVelocity();
    bottomAppliedVolts = bottomFlywheelMotor.getMotorVoltage();
    bottomStatorCurrent = bottomFlywheelMotor.getStatorCurrent();
    bottomTemperature = bottomFlywheelMotor.getDeviceTemp();
    bottomSupplyCurrent = bottomFlywheelMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, topVelocity, topTemperature, topSupplyCurrent, topStatorCurrent, topAppliedVolts,
          bottomVelocity, bottomTemperature, bottomSupplyCurrent, bottomStatorCurrent, bottomAppliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll
      (topVelocity, topTemperature, topStatorCurrent, topSupplyCurrent, topAppliedVolts, 
        bottomVelocity, bottomTemperature, bottomSupplyCurrent, bottomStatorCurrent, bottomAppliedVolts);

    if(beamBreak.get())
    {
      beamBreakTimer.reset();
    }

    super.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    super.topStatorCurrentAmps = topStatorCurrent.getValueAsDouble();
    super.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
    super.topVelocity = topVelocity.getValueAsDouble();
    super.topTempCelsius = topTemperature.getValueAsDouble();
    super.topDesiredVelocity = desiredVelocity;

    super.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    super.bottomStatorCurrentAmps = bottomStatorCurrent.getValueAsDouble();
    super.bottomSupplyCurrentAmps = bottomSupplyCurrent.getValueAsDouble();
    super.bottomVelocity = bottomVelocity.getValueAsDouble();
    super.bottomTempCelsius = bottomTemperature.getValueAsDouble();
    super.bottomDesiredVelocity = desiredVelocity;

    super.topIsNoteDetected = beamBreakDebouncer.calculate(!beamBreak.get());
    super.bottomIsNoteDetected = beamBreakDebouncer.calculate(!beamBreak.get());
    super.state = currentState; 

    // top motor logging
    DogLog.log("Flywheels/Top/Velocity", super.topVelocity);
    DogLog.log("Flywheels/Top/AppliedVoltage", super.topAppliedVolts);
    DogLog.log("Flywheels/Top/TempCelcius", super.topTempCelsius);
    DogLog.log("Flywheels/Top/StatorCurrentAmps", super.topStatorCurrentAmps);
    DogLog.log("Flywheels/Top/SupplyCurrentAmps", super.topSupplyCurrentAmps);
    DogLog.log("Flywheels/Top/NoteDetected", super.topIsNoteDetected);

    // bottom motor logging
    DogLog.log("Flywheels/Bottom/Velocity", super.bottomVelocity);
    DogLog.log("Flywheels/Bottom/AppliedVoltage", super.bottomAppliedVolts);
    DogLog.log("Flywheels/Bottom/TempCelcius", super.bottomTempCelsius);
    DogLog.log("Flywheels/Bottom/StatorCurrentAmps", super.bottomStatorCurrentAmps);
    DogLog.log("Flywheels/Bottom/SupplyCurrentAmps", super.bottomSupplyCurrentAmps);
    DogLog.log("Flywheels/Bottom/NoteDetected", super.bottomIsNoteDetected);
    
    DogLog.log("IntakeRollers/State", super.state.toString());
    DogLog.log("IntakeRollers/BeamBreak", beamBreak.get());
  }

  @Override
  public void stop() {
    topFlywheelMotor.stopMotor();
    bottomFlywheelMotor.stopMotor();
  }
  
  @Override
  public void setVelocity(ShooterFlywheelsStates state) {
    double velocity = SubsystemUtil.shooterFlywheelStateToVelocity(state);
    topFlywheelMotor.set(velocity);
    bottomFlywheelMotor.set(-velocity);
  }
}