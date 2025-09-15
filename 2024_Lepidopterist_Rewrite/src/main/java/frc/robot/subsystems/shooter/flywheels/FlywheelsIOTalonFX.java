package frc.robot.subsystems.shooter.flywheels;
import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

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

public class FlywheelsIOTalonFX extends FlywheelsIO{
  
  private final TalonFX topFlywheelMotor;
  private final TalonFX bottomFlywheelMotor;
  TalonFXConfiguration topFlywheelConfig;
  TalonFXConfiguration bottomFlywheelConfig;

  private double wantedVelocity;

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
  
  public FlywheelsIOTalonFX() {
    topFlywheelMotor = new TalonFX(topMotorID, topMotorCANBus);
    bottomFlywheelMotor = new TalonFX(bottomMotorID, bottomMotorCANBus);
    
    topFlywheelConfig.Slot0.kP = topRealP;
    topFlywheelConfig.Slot0.kI = topRealI;
    topFlywheelConfig.Slot0.kD = topRealD;
    topFlywheelConfig.Slot0.kS = topRealS;
    topFlywheelConfig.Slot0.kV = topRealV;

    topFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    topFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    topFlywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    topFlywheelConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    bottomFlywheelConfig.Slot0.kP = bottomRealP;
    bottomFlywheelConfig.Slot0.kI = bottomRealI;
    bottomFlywheelConfig.Slot0.kD = bottomRealD;
    bottomFlywheelConfig.Slot0.kS = bottomRealS;
    bottomFlywheelConfig.Slot0.kV = bottomRealV;

    bottomFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = EnableCurrentLimit;
    bottomFlywheelConfig.CurrentLimits.SupplyCurrentLimit = ContinousCurrentLimit;
    bottomFlywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = PeakCurrentLimit;
    bottomFlywheelConfig.CurrentLimits.SupplyCurrentLowerTime = PeakCurrentDuration;

    topFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);
    topFlywheelMotor.optimizeBusUtilization();
    
    bottomFlywheelMotor.setNeutralMode(NeutralModeValue.Brake);
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
    BaseStatusSignal.refreshAll(topVelocity, topTemperature, topSupplyCurrent, topStatorCurrent, topAppliedVolts,
    bottomVelocity, bottomTemperature, bottomSupplyCurrent, bottomStatorCurrent, bottomAppliedVolts);
    
    super.topAppliedVoltage = topAppliedVolts.getValueAsDouble();
    super.topVelocity = topVelocity.getValueAsDouble();
    super.topStatorCurrentAmps = topStatorCurrent.getValueAsDouble();
    super.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
    super.topTempCelsius = topTemperature.getValueAsDouble();

    super.bottomAppliedVoltage = bottomAppliedVolts.getValueAsDouble();
    super.bottomVelocity = bottomVelocity.getValueAsDouble();
    super.bottomStatorCurrentAmps = bottomStatorCurrent.getValueAsDouble();
    super.bottomSupplyCurrentAmps = bottomSupplyCurrent.getValueAsDouble();
    super.bottomTempCelsius = bottomTemperature.getValueAsDouble();
   
    super.wantedVelocity = wantedVelocity;

    DogLog.log("Shooter/Flywheels/Top Flywheel/Velocity", super.topVelocity);
    DogLog.log("Shooter/Flywheels/Top Flywheel/AppliedVoltage", super.topAppliedVoltage);
    DogLog.log("Shooter/Flywheels/Top Flywheel/TempCelcius", super.topTempCelsius);
    DogLog.log("Shooter/Flywheels/Top Flywheel/StatorCurrentAmps", super.topStatorCurrentAmps);
    DogLog.log("Shooter/Flywheels/Top Flywheel/SupplyCurrentAmps", super.topSupplyCurrentAmps);

    DogLog.log("Shooter/Flywheels/Bottom Flywheel/Velocity", super.bottomVelocity);
    DogLog.log("Shooter/Flywheels/Bottom Flywheel/AppliedVoltage", super.bottomAppliedVoltage);
    DogLog.log("Shooter/Flywheels/Bottom Flywheel/TempCelcius", super.bottomTempCelsius);
    DogLog.log("Shooter/Flywheels/Bottom Flywheel/StatorCurrentAmps", super.bottomStatorCurrentAmps);
    DogLog.log("Shooter/Flywheels/Bottom Flywheel/SupplyCurrentAmps", super.bottomSupplyCurrentAmps);
    
    DogLog.log("Shooter/Flywheels/Top Flywheel/isAtSpeed", super.isTopFlywheelAtSpeed);
    DogLog.log("Shooter/Flywheels/Top Flywheel/isAtSpeed", super.isBottomFlywheelAtSpeed);
  }

  @Override
  public void stop() {
    setVelocity(0);
  }

  @Override
  public void setVelocity(double velocity) {
    topFlywheelMotor.set(velocity);
    bottomFlywheelMotor.set(velocity);
  }
 }

