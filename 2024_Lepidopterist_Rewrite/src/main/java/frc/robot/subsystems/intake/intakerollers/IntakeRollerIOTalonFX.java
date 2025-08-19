package frc.robot.subsystems.intake.intakerollers;

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
import frc.robot.subsystems.intake.intakerollers.IntakeRollerConstants.IntakeRollerStates;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SubsystemUtil;

public class IntakeRollerIOTalonFX extends IntakeRollerIO {
  private final TalonFX intakeRollerMotor;
  TalonFXConfiguration indexerConfig;
  Debouncer beamBreakDebouncer = new Debouncer(beamBreakDebounce);
  private DigitalInput beamBreak;
  private IntakeRollerConstants.IntakeRollerStates currentState = IntakeRollerStates.STOW;

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;

  private double desiredVelocity;

  public IntakeRollerIOTalonFX() {
    intakeRollerMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
    beamBreak = new DigitalInput(IntakeRollerConstants.beamBreakPort);

    indexerConfig = new TalonFXConfiguration();

    // Basic wrist motor config setup
    intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    // PID
    indexerConfig.Slot0.kP = realP;
    indexerConfig.Slot0.kI = realI;
    indexerConfig.Slot0.kD = realD;
    indexerConfig.Slot0.kS = realS;
    indexerConfig.Slot0.kV = realV;

    // Debouncer for beambreak
    beamBreakDebouncer.setDebounceType(DebounceType.kFalling);

    // setting the actual loggging variables for DogLog & tryna make sure the motor actually uses our custom config
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
    // constanly updating the actual loggging variables for DogLog
    BaseStatusSignal.refreshAll(velocity, temperature, statorCurrent, supplyCurrent, appliedVolts);
    super.appliedVolts = appliedVolts.getValueAsDouble();
    super.statorCurrentAmps = statorCurrent.getValueAsDouble();
    super.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    super.velocity = velocity.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
    super.desiredVelocity = desiredVelocity;
    super.isNoteDetected = beamBreakDebouncer.calculate(!beamBreak.get());
    super.state = currentState; 

    // basic logging for the motor
    DogLog.log("IntakeRollers/Velocity", super.velocity);
    DogLog.log("IntakeRollers/AppliedVoltage", super.appliedVolts);
    DogLog.log("IntakeRollers/TempCelcius", super.tempCelsius);
    DogLog.log("IntakeRollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("IntakeRollers/SupplyCurrentAmps", super.supplyCurrentAmps);
    DogLog.log("IntakeRollers/State", super.state.toString());

    // basic logging for the beambreak
    DogLog.log("IntakeRollers/NoteDetected", super.isNoteDetected);
    DogLog.log("IntakeRollers/BeamBreak", beamBreak.get());
  }

  // stop function to stop the motor when we want
  @Override
  public void stop() {
    intakeRollerMotor.stopMotor();
  }

  // Updates the velocity of the motor depending on the state we are moving to
  @Override
  public void setVelocity(IntakeRollerStates state) {
    double velocity = SubsystemUtil.intakeRollerStateToVelocity(state);
    intakeRollerMotor.set(velocity);
  }
}