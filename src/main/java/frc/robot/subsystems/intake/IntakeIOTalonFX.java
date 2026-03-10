package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
    private final IntakeConfig config;
    private final TalonFX rollerMotor;
    private final TalonFX pivotMotor;

    private final StatusSignal<AngularVelocity> rollerVelocityStatusSignal;
    private final StatusSignal<Voltage> rollerMotorVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerTemperature;

    private final StatusSignal<Angle> pivotPositionStatusSignal;
    private final StatusSignal<AngularVelocity> pivotVelocityStatusSignal;
    private final StatusSignal<Voltage> pivotMotorVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotTemperature;

    private final VelocityVoltage rollerMotorRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage pivotMotorRequest = new PositionVoltage(0).withSlot(0);

    private final TalonFXConfiguration rollerConfig;
    private final TalonFXConfiguration pivotConfig;

    public IntakeIOTalonFX(IntakeConfig config) {
        this.config = config;
        rollerConfig = new TalonFXConfiguration();

        rollerConfig.Slot0.kP = config.rollerKP;
        rollerConfig.Slot0.kI = config.rollerKI;
        rollerConfig.Slot0.kD = config.rollerKD;
        rollerConfig.Slot0.kS = config.rollerKS;
        rollerConfig.Slot0.kV = config.rollerKV;
        rollerConfig.Slot0.kA = config.rollerKA;
        rollerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        rollerConfig.ClosedLoopGeneral.ContinuousWrap = false;
        rollerConfig.Feedback.SensorToMechanismRatio = config.rollerMotorToOutputShaftRatio;

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = config.isRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = config.rollerStatorCurrentLimit;

        rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.rollerPeakForwardTorqueCurrent;
        rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.rollerPeakReverseTorqueCurrent;

        rollerConfig.FutureProofConfigs = false;

        rollerMotor = new TalonFX(config.rollerCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));

        pivotConfig = new TalonFXConfiguration();

        pivotConfig.Slot0.kP = config.pivotKP;
        pivotConfig.Slot0.kI = config.pivotKI;
        pivotConfig.Slot0.kD = config.pivotKD;
        pivotConfig.Slot0.kS = config.pivotKS;
        pivotConfig.Slot0.kV = config.pivotKV;
        pivotConfig.Slot0.kA = config.pivotKA;
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        pivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
        pivotConfig.Feedback.SensorToMechanismRatio = config.pivotMotorToOutputShaftRatio;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = config.isPivotInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = config.pivotStatorCurrentLimit;

        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.pivotPeakForwardTorqueCurrent;
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.pivotPeakReverseTorqueCurrent;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.pivotMaxAngleRotations;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.pivotMinAngleRotations;

        pivotConfig.FutureProofConfigs = false;

        pivotMotor = new TalonFX(config.pivotCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.setPosition(config.pivotStartingAngleRotations, 0.25));

        rollerTorqueCurrent = rollerMotor.getTorqueCurrent().clone();
        rollerTemperature = rollerMotor.getDeviceTemp().clone();
        rollerVelocityStatusSignal = rollerMotor.getVelocity().clone();
        rollerMotorVoltage = rollerMotor.getMotorVoltage().clone();

        pivotTorqueCurrent = pivotMotor.getTorqueCurrent().clone();
        pivotTemperature = pivotMotor.getDeviceTemp().clone();
        pivotPositionStatusSignal = pivotMotor.getPosition().clone();
        pivotVelocityStatusSignal = pivotMotor.getVelocity().clone();
        pivotMotorVoltage = pivotMotor.getMotorVoltage().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            rollerTorqueCurrent, rollerTemperature,
            rollerVelocityStatusSignal, rollerMotorVoltage,
            pivotTorqueCurrent, pivotTemperature,
            pivotPositionStatusSignal, pivotVelocityStatusSignal, pivotMotorVoltage);

        rollerMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            rollerTorqueCurrent, rollerTemperature,
            rollerVelocityStatusSignal, rollerMotorVoltage,
            pivotTorqueCurrent, pivotTemperature,
            pivotPositionStatusSignal, pivotVelocityStatusSignal, pivotMotorVoltage);

        inputs.rollerVelocityRotationsPerSec = rollerVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.rollerAppliedVolts = rollerMotorVoltage.getValue().in(Volts);
        inputs.rollerTorqueCurrent = rollerTorqueCurrent.getValue().in(Amps);
        inputs.rollerTemperatureFahrenheit = rollerTemperature.getValue().in(Fahrenheit);

        inputs.pivotAngleRotations = pivotPositionStatusSignal.getValue().in(Rotations);
        inputs.pivotVelocityRotationsPerSec = pivotVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.pivotAppliedVolts = pivotMotorVoltage.getValue().in(Volts);
        inputs.pivotTorqueCurrent = pivotTorqueCurrent.getValue().in(Amps);
        inputs.pivotTemperatureFahrenheit = pivotTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        rollerMotor.setControl(rollerMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setVoltage(double voltage) {
        rollerMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPivotAngle(double angleRotations) {
        double clampedAngle = Math.max(config.pivotMinAngleRotations,
            Math.min(config.pivotMaxAngleRotations, angleRotations));
        pivotMotor.setControl(pivotMotorRequest.withPosition(clampedAngle));
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        rollerConfig.Slot0.kP = config.kP();
        rollerConfig.Slot0.kI = config.kI();
        rollerConfig.Slot0.kD = config.kD();
        rollerConfig.Slot0.kS = config.kS();
        rollerConfig.Slot0.kV = config.kV();
        rollerConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerConfig, 0.25));
    }

    @Override
    public void configurePivotControlLoop(MotorControlLoopConfig config) {
        pivotConfig.Slot0.kP = config.kP();
        pivotConfig.Slot0.kI = config.kI();
        pivotConfig.Slot0.kD = config.kD();
        pivotConfig.Slot0.kS = config.kS();
        pivotConfig.Slot0.kV = config.kV();
        pivotConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    }
}
