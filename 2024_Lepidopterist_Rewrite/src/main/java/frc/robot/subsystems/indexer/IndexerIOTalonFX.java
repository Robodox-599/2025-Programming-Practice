package frc.robot.subsystems.indexer;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;


public class IndexerIOTalonFX extends IndexerIO{
  
  private final TalonFX indexerMotor;
  TalonFXConfiguration rollersConfig;
  private DigitalInput m_beamBreak;
  private double wantedVelocity;
  Debouncer noteDebouncer = new Debouncer(beamBreakDebounce);
  Debouncer ensureNoteDebouncer = new Debouncer(ensureNoteDebounce);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;
  
  public IndexerIOTalonFX() {
    indexerMotor = new TalonFX(indexerMotorID, indexerMotorCANBus);
    TalonFXConfiguration indexerConfig;
    indexerConfig = new TalonFXConfiguration();
   
    m_beamBreak = new DigitalInput(0);
    Debouncer noteDebouncer = new Debouncer(beamBreakDebounce);
    Debouncer ensureNoteDebouncer = new Debouncer(ensureNoteDebounce);

    rollersConfig.Slot0.kP = realP;
    rollersConfig.Slot0.kI = realI;
    rollersConfig.Slot0.kD = realD;
    rollersConfig.Slot0.kS = realS;
    rollersConfig.Slot0.kV = realV;

    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    indexerMotor.setNeutralMode(NeutralModeValue.Brake);
    indexerMotor.optimizeBusUtilization();
    
    velocity = indexerMotor.getVelocity();
    appliedVolts = indexerMotor.getMotorVoltage();
    statorCurrent = indexerMotor.getStatorCurrent();
    temperature = indexerMotor.getDeviceTemp();
    supplyCurrent = indexerMotor.getSupplyCurrent();
    
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, temperature, supplyCurrent, statorCurrent, appliedVolts);
  }

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(velocity, temperature, statorCurrent, supplyCurrent, appliedVolts);
    
    super.appliedVoltage = appliedVolts.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.statorCurrentAmps = statorCurrent.getValueAsDouble();
    super.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.noteDetected = noteDebouncer.calculate(!m_beamBreak.get());
    super.noteEnsured = ensureNoteDebouncer.calculate(!m_beamBreak.get());
   
    super.wantedVelocity = wantedVelocity;

    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVoltage);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/SupplyCurrentAmps", super.supplyCurrentAmps);
    DogLog.log("Rollers/noteDetected", super.noteDetected);
    DogLog.log("Rollers/noteEnsured", super.noteEnsured);
    DogLog.log("Rollers/BeamBreak", m_beamBreak.get());
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    indexerMotor.set(velocity);
  }
 }

