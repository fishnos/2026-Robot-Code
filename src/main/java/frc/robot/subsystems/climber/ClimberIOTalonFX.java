package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.configs.ClimberConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climberMotor;

    private final StatusSignal<Angle> climberPositionStatusSignal;
    private final StatusSignal<AngularVelocity> climberVelocityStatusSignal;
    private final StatusSignal<Voltage> climberMotorVoltage;
    private final StatusSignal<Current> climberTorqueCurrent;
    private final StatusSignal<Temperature> climberTemperature;

    private final MotionMagicVoltage climberMotorRequest = new MotionMagicVoltage(0).withSlot(0);

    private final ClimberConfig config;
    private final TalonFXConfiguration climberConfig;

    public ClimberIOTalonFX(ClimberConfig config) {
        this.config = config;
        climberConfig = new TalonFXConfiguration();

        climberConfig.Slot0.kP = config.climberKP;
        climberConfig.Slot0.kI = config.climberKI;
        climberConfig.Slot0.kD = config.climberKD;
        climberConfig.Slot0.kS = config.climberKS;
        climberConfig.Slot0.kV = config.climberKV;
        climberConfig.Slot0.kA = config.climberKA;
        climberConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        climberConfig.ClosedLoopGeneral.ContinuousWrap = false;
        climberConfig.Feedback.SensorToMechanismRatio = config.climberMotorToOutputShaftRatio;

        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.MotorOutput.Inverted = config.isClimberInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = config.climberStatorCurrentLimit;

        climberConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.climberPeakForwardTorqueCurrent;
        climberConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.climberPeakReverseTorqueCurrent;

        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.climberMaxPositionRotations;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.climberMinPositionRotations;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = config.climberMaxVelocityRotationsPerSec;
        climberConfig.MotionMagic.MotionMagicAcceleration = config.climberMaxAccelerationRotationsPerSec2;

        climberConfig.FutureProofConfigs = false;

        climberMotor = new TalonFX(config.climberCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.setPosition(config.climberStartingPositionRotations, 0.25));

        climberPositionStatusSignal = climberMotor.getPosition().clone();
        climberVelocityStatusSignal = climberMotor.getVelocity().clone();
        climberMotorVoltage = climberMotor.getMotorVoltage().clone();
        climberTorqueCurrent = climberMotor.getTorqueCurrent().clone();
        climberTemperature = climberMotor.getDeviceTemp().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            climberPositionStatusSignal, climberVelocityStatusSignal,
            climberMotorVoltage, climberTorqueCurrent, climberTemperature);

        climberMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            climberPositionStatusSignal, climberVelocityStatusSignal,
            climberMotorVoltage, climberTorqueCurrent, climberTemperature);

        inputs.positionRotations = climberPositionStatusSignal.getValue().in(Rotations);
        inputs.velocityRotationsPerSec = climberVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.appliedVolts = climberMotorVoltage.getValue().in(Volts);
        inputs.torqueCurrent = climberTorqueCurrent.getValue().in(Amps);
        inputs.temperatureFahrenheit = climberTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setPosition(double positionRotations) {
        double clampedPosition = Math.max(config.climberMinPositionRotations,
            Math.min(config.climberMaxPositionRotations, positionRotations));
        climberMotor.setControl(climberMotorRequest.withPosition(clampedPosition));
    }

    @Override
    public void setVoltage(double voltage) {
        climberMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        climberConfig.Slot0.kP = config.kP();
        climberConfig.Slot0.kI = config.kI();
        climberConfig.Slot0.kD = config.kD();
        climberConfig.Slot0.kS = config.kS();
        climberConfig.Slot0.kV = config.kV();
        climberConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));
    }

    @Override
    public void enableEStop() {
        climberConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));
    }

    @Override
    public void disableEStop() {
        climberConfig.CurrentLimits.StatorCurrentLimit = config.climberStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(climberConfig, 0.25));
    }
}
