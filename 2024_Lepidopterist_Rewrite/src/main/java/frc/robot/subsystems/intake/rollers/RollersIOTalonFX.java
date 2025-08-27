package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.indexer.IndexerConstants.ContinousCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.EnableCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.PeakCurrentDuration;
import static frc.robot.subsystems.indexer.IndexerConstants.PeakCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.realD;
import static frc.robot.subsystems.indexer.IndexerConstants.realI;
import static frc.robot.subsystems.indexer.IndexerConstants.realP;
import static frc.robot.subsystems.indexer.IndexerConstants.realS;
import static frc.robot.subsystems.indexer.IndexerConstants.realV;
import static frc.robot.subsystems.intake.rollers.RollersConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollersIOTalonFX extends RollersIO{
  
  private final TalonFX rollersMotor;
  TalonFXConfiguration rollersConfig;


  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temperature;
  
  public RollersIOTalonFX() {
    rollersMotor = new TalonFX(rollersMotorID, rollersMotorCANBus);
       
    rollersConfig.Slot0.kP = realP;
    rollersConfig.Slot0.kI = realI;
    rollersConfig.Slot0.kD = realD;
    rollersConfig.Slot0.kS = realS;
    rollersConfig.Slot0.kV = realV;

    rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    rollersConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    rollersMotor.setNeutralMode(NeutralModeValue.Brake);
    rollersMotor.optimizeBusUtilization();
    
    velocity = rollersMotor.getVelocity();
    appliedVolts = rollersMotor.getMotorVoltage();
    statorCurrent = rollersMotor.getStatorCurrent();
    temperature = rollersMotor.getDeviceTemp();
    supplyCurrent = rollersMotor.getSupplyCurrent();
    
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
   
    DogLog.log("Rollers/Velocity", super.velocity);
    DogLog.log("Rollers/AppliedVoltage", super.appliedVoltage);
    DogLog.log("Rollers/TempCelcius", super.tempCelsius);
    DogLog.log("Rollers/StatorCurrentAmps", super.statorCurrentAmps);
    DogLog.log("Rollers/SupplyCurrentAmps", super.supplyCurrentAmps);
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    rollersMotor.set(velocity);
  }
 }


