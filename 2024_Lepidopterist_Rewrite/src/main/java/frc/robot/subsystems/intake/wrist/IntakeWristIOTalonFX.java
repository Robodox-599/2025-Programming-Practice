package frc.robot.subsystems.intake.wrist;
import static frc.robot.subsystems.intake.wrist.IntakeWristConstants.*;

import dev.doglog.DogLog;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Temperature;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class IntakeWristIOTalonFX extends IntakeWristIO {

  private final TalonFX wristMotor;
  TalonFXConfiguration wristConfig;

  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Angle> position;

  private final StatusSignal<Angle> absolutePosition;

  private final MotionMagicVoltage m_request;
  private final CANcoder cancoder;

  public IntakeWristIOTalonFX() {

    wristMotor = new TalonFX(wristMotorID, wristMotorCANBus);
    wristConfig = new TalonFXConfiguration();
    cancoder = new CANcoder(cancoderID, wristMotorCANBus);
    m_request = new MotionMagicVoltage(null);

    wristConfig.Slot0.kP = realkP;
    wristConfig.Slot0.kI = realkI;
    wristConfig.Slot0.kD = realkD;
    wristConfig.Slot0.kV = realkV;
    wristConfig.Slot0.kS = realkS;
    wristConfig.Slot0.kG = realkG;

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    wristConfig.Feedback.RotorToSensorRatio = gearRatio;
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;

    appliedVoltage = wristMotor.getMotorVoltage();
    velocity = wristMotor.getVelocity();
    position = wristMotor.getPosition();
    temperature = wristMotor.getDeviceTemp();
    current = wristMotor.getStatorCurrent();
    absolutePosition = cancoder.getAbsolutePosition();

    wristMotor.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVoltage, velocity, temperature, position, current, absolutePosition);
  } 

  @Override
  public void updateInputs() {
    BaseStatusSignal.refreshAll(
        appliedVoltage, velocity, temperature, position, current, absolutePosition);
    super.appliedVoltage = appliedVoltage.getValueAsDouble();
    super.currentAmps = current.getValueAsDouble();
    super.velocity = velocity.getValueAsDouble();
    super.currentPosition = position.getValueAsDouble();
    super.tempCelsius = temperature.getValueAsDouble();
   
    DogLog.log("Intake/Wrist/Velocity", super.velocity);
    DogLog.log("Intake/Wrist/CurrentAmps", super.currentAmps);
    DogLog.log("Intake/Wrist/Temperature", super.tempCelsius);
    DogLog.log("Intake/Wrist/TargetPosition", targetPosition);
    DogLog.log("Intake/Wrist/AppliedVoltage", super.appliedVoltage);
    DogLog.log("Intake/Wrist/CurrentPosition", super.currentPosition);
    DogLog.log("Intake/Wrist/AbsolutePosition", absolutePosition.getValueAsDouble());
  }
  @Override
  public void goToAngle(double angle) {
    super.targetPosition = angle;
    wristMotor.setControl(m_request);
    m_request.withPosition(angle);
  }
  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }
  @Override
  public void setBrake(boolean brake) {
    wristMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
  @Override
  public void stop() {
    wristMotor.stopMotor();
  }
}

