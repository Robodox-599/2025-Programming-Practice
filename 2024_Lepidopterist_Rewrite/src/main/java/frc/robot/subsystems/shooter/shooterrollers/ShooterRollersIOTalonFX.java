package frc.robot.subsystems.shooter.shooterrollers;

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
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollersConstants.ShooterRollersStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ShooterRollersIOTalonFX extends ShooterRollersIO {
  private final TalonFX indexerMotor;
  TalonFXConfiguration indexerConfig;
  Debouncer algaeStallDebouncer = new Debouncer(algaeDebounce);
  Debouncer noteBeamBreakDebouncer = new Debouncer(beamBreakDebounce);
  Debouncer ensureNoteBeamBreakDebouncer = new Debouncer(ensureCoralDebounce);
  private DigitalInput m_BeamBreak2;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public ShooterRollersIOTalonFX() {
    indexerMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    m_BeamBreak2 = new DigitalInput(ShooterRollersConstants.beakBreakPort);

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

    noteBeamBreakDebouncer.setDebounceType(DebounceType.kFalling);
    ensureNoteBeamBreakDebouncer.setDebounceType(DebounceType.kFalling);

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
    super.isNoteDetected = noteBeamBreakDebouncer.calculate(!m_BeamBreak2.get());
    super.isNoteEnsured = ensureNoteBeamBreakDebouncer.calculate(!m_BeamBreak2.get());

    DogLog.log("Indexer/Velocity", super.velocity);
    DogLog.log("Indexer/AppliedVoltage", super.appliedVolts);
    DogLog.log("Indexer/TempCelcius", super.tempCelsius);
    DogLog.log("Indexer/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Indexer/SupplyCurrentAmps", super.supplyCurrentAmps);

    DogLog.log("Indexer/NoteDetected", super.isNoteDetected);
    DogLog.log("Rollers/NoteEnsured", super.isNoteEnsured);
    DogLog.log("Indexer/BeamBreak", m_BeamBreak2.get());
  }

  @Override
  public void stop() {
    setVelocity(ShooterRollersStates.STOPPED);
  }

  @Override
  public void setVelocity(ShooterRollersStates state) {
    double velocity = SubsystemUtil.shooterRollersStateToVelocity(state);
    indexerMotor.set(velocity);
  }
}