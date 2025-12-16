package frc.robot.subsystems.endefectorwrist;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class EndefectorWristIOTalonFX extends EndefectorWristIO {
    private final TalonFX endefectorWristMotor;
    TalonFXConfiguration endefectorWristConfig;
    MotionMagicVoltage m_request;
    private CANcoder cancoder;
    CANcoderConfiguration cancoderConfig;

    private final StatusSignal<AngularVelocity> endefectorWristVelocityRad;
    private final StatusSignal<Temperature> endefectorWristTemperature;
    private final StatusSignal<Angle> endefectorWristPosition;
    private final StatusSignal<Voltage> endefectorWristAppliedVolts;
    private final StatusSignal<Current> endefectorWristStatorCurrent;
    private final StatusSignal<Current> endefectorWristSupplyCurrent;

    public EndefectorWristIOTalonFX() {
        endefectorWristMotor = new TalonFX(EndefectorWristConstants.endefectorWristMotorID, EndefectorWristConstants.endefectorWristCANBus);
        endefectorWristConfig = new TalonFXConfiguration();
        m_request = new MotionMagicVoltage(0);
        cancoder = new CANcoder(EndefectorWristConstants.cancoderID);
        cancoderConfig = new CANcoderConfiguration();
        
        endefectorWristConfig.Slot0.kP = EndefectorWristConstants.kP;
        endefectorWristConfig.Slot0.kI = EndefectorWristConstants.kI;
        endefectorWristConfig.Slot0.kD = EndefectorWristConstants.kD;
        endefectorWristConfig.Slot0.kS = EndefectorWristConstants.kS;
        endefectorWristConfig.Slot0.kV = EndefectorWristConstants.kV;
        endefectorWristConfig.Slot0.kG = EndefectorWristConstants.kG;
        
        endefectorWristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        endefectorWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        endefectorWristConfig.CurrentLimits.SupplyCurrentLimit = EndefectorWristConstants.supplyCurrentLimit;

        endefectorWristConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        endefectorWristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        endefectorWristConfig.Feedback.RotorToSensorRatio = EndefectorWristConstants.gearRatio;
        endefectorWristConfig.ClosedLoopGeneral.ContinuousWrap = false;

        endefectorWristConfig.MotionMagic.MotionMagicCruiseVelocity = EndefectorWristConstants.maxVelocity;
        endefectorWristConfig.MotionMagic.MotionMagicAcceleration = EndefectorWristConstants.maxAcceleration;

        cancoderConfig.MagnetSensor.MagnetOffset = EndefectorWristConstants.cancoderMagnetOffset;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = EndefectorWristConstants.discontinuityPoint; 

        endefectorWristMotor.getConfigurator().apply(endefectorWristConfig);
        endefectorWristMotor.setNeutralMode(NeutralModeValue.Brake);
        cancoder.getConfigurator().apply(cancoderConfig);

        endefectorWristVelocityRad = endefectorWristMotor.getVelocity();
        endefectorWristTemperature = endefectorWristMotor.getDeviceTemp();
        endefectorWristAppliedVolts = endefectorWristMotor.getMotorVoltage();
        endefectorWristPosition = endefectorWristMotor.getPosition();
        endefectorWristStatorCurrent = endefectorWristMotor.getStatorCurrent();
        endefectorWristSupplyCurrent = endefectorWristMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
        endefectorWristVelocityRad, endefectorWristTemperature, endefectorWristAppliedVolts, endefectorWristPosition,
        endefectorWristStatorCurrent, endefectorWristSupplyCurrent);

        endefectorWristMotor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(endefectorWristVelocityRad, endefectorWristTemperature, endefectorWristAppliedVolts, endefectorWristPosition,
        endefectorWristStatorCurrent, endefectorWristSupplyCurrent);
        super.position = endefectorWristPosition.getValueAsDouble();
        super.velocity = endefectorWristVelocityRad.getValueAsDouble();
        super.isWristInPosition = wantedPosition == position;

        DogLog.log("EndefectorWrist/Position", super.position);
        DogLog.log("EndefectorWrist/Velocity", super.velocity);
        DogLog.log("EndefectorWrist/isWristInPosition", super.isWristInPosition);
    }

    @Override
    public void setVelocity(double velocity) {
        endefectorWristMotor.set(velocity);
    }

    @Override
    public void stop() {
        endefectorWristMotor.stopMotor();
    }

    @Override
    public void setPosition(double position) {
        endefectorWristMotor.setControl(m_request.withPosition(position));
        position = wantedPosition;
    }
//0.5 0.33

}
