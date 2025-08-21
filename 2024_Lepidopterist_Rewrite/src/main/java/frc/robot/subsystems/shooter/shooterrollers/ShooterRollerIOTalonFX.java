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
import frc.robot.subsystems.shooter.shooterrollers.ShooterRollerConstants.ShooterRollerStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class ShooterRollerIOTalonFX extends ShooterRollerIO {
  private final TalonFX intakeRollerMotor;
  TalonFXConfiguration indexerConfig;
  Debouncer beamBreakDebouncer = new Debouncer(beamBreakDebounce);
  private DigitalInput beamBreak;
  private ShooterRollerConstants.ShooterRollerStates currentState = ShooterRollerStates.STOW;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public ShooterRollerIOTalonFX() {
    intakeRollerMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    beamBreak = new DigitalInput(ShooterRollerConstants.beamBreakPort);

    indexerConfig = new TalonFXConfiguration();

    intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    indexerConfig.Slot0.kP = realP;
    indexerConfig.Slot0.kI = realI;
    indexerConfig.Slot0.kD = realD;
    indexerConfig.Slot0.kS = realS;
    indexerConfig.Slot0.kV = realV;

    beamBreakDebouncer.setDebounceType(DebounceType.kFalling);

    PhoenixUtil.tryUntilOk(10, () -> intakeRollerMotor.getConfigurator().apply(indexerConfig, 1));
    intakeRollerMotor.optimizeBusUtilization();

    velocity = intakeRollerMotor.getVelocity();
    appliedVolts = intakeRollerMotor.getMotorVoltage();
    statorCurrent = intakeRollerMotor.getStatorCurrent();
    temperature = intakeRollerMotor.getDeviceTemp();
    supplyCurrent = intakeRollerMotor.getSupplyCurrent();
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

    DogLog.log("ShooterRollers/Velocity", super.velocity);
    DogLog.log("ShooterRollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("ShooterRollers/TempCelcius", super.tempCelsius);
    DogLog.log("ShooterRollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("ShooterRollers/SupplyCurrentAmps", super.supplyCurrentAmps);
    DogLog.log("ShooterRollers/State", super.state.toString());

    DogLog.log("ShooterRollers/NoteDetected", super.isNoteDetected);
    DogLog.log("ShooterRollers/BeamBreak", beamBreak.get());
  }

  @Override
  public void stop() {
    intakeRollerMotor.stopMotor();
  }
  
  @Override
  public void setVelocity(ShooterRollerStates state) {
    double velocity = SubsystemUtil.shooterRollerStateToVelocity(state);
    intakeRollerMotor.set(velocity);
  }
}