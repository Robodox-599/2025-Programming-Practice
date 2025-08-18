package frc.robot.subsystems.indexer;

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
import frc.robot.subsystems.indexer.IndexerConstants.IndexerStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class IndexerIOTalonFX extends IndexerIO {
  private final TalonFX indexerMotor;
  TalonFXConfiguration indexerConfig;
  Debouncer beamBreakDebouncer = new Debouncer(beamBreakDebounce);
  private DigitalInput beamBreak;
  private IndexerConstants.IndexerStates currentState = IndexerStates.STOW;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public IndexerIOTalonFX() {
    indexerMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    beamBreak = new DigitalInput(IndexerConstants.beamBreakPort);

    indexerConfig = new TalonFXConfiguration();

    indexerMotor.setNeutralMode(NeutralModeValue.Brake);

    indexerConfig.Slot0.kP = realP;
    indexerConfig.Slot0.kI = realI;
    indexerConfig.Slot0.kD = realD;
    indexerConfig.Slot0.kS = realS;
    indexerConfig.Slot0.kV = realV;

    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    beamBreakDebouncer.setDebounceType(DebounceType.kFalling);

    PhoenixUtil.tryUntilOk(10, () -> indexerMotor.getConfigurator().apply(indexerConfig, 1));
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
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.statorCurrentAmps = statorCurrent.getValueAsDouble();
    super.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.desiredVelocity = desiredVelocity;
    super.isNoteDetected = beamBreakDebouncer.calculate(!beamBreak.get());
    super.state = currentState; 

    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/AppliedVoltage", super.appliedVolts);
    DogLog.log("Indexer/TempCelcius", super.tempCelsius);
    DogLog.log("Indexer/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Indexer/SupplyCurrentAmps", super.supplyCurrentAmps);
    DogLog.log("Indexer/State", super.state.toString());

    DogLog.log("Indexer/NoteDetected", super.isNoteDetected);
    DogLog.log("Indexer/BeamBreak", beamBreak.get());
  }

  @Override
  public void stop() {
    indexerMotor.stopMotor();
  }

  @Override
  public void setVelocity(IndexerStates state) {
    double velocity = SubsystemUtil.indexerStateToVelocity(state);
    indexerMotor.set(velocity);
  }
}