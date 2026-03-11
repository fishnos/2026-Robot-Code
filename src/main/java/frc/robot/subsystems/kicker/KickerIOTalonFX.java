package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.configs.KickerConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class KickerIOTalonFX implements KickerIO {
    private final KickerConfig config;
    private final TalonFX kickerMotor;

    private final StatusSignal<AngularVelocity> kickerVelocityStatusSignal;
    private final StatusSignal<Voltage> kickerMotorVoltage;
    private final StatusSignal<Current> kickerTorqueCurrent;
    private final StatusSignal<Temperature> kickerTemperature;

    private final VelocityVoltage kickerMotorRequest = new VelocityVoltage(0).withSlot(0);

    private final TalonFXConfiguration kickerConfig;

    public KickerIOTalonFX(KickerConfig config) {
        this.config = config;
        // Kicker motor configuration (velocity control)
        kickerConfig = new TalonFXConfiguration();

        kickerConfig.Slot0.kP = config.kickerKP;
        kickerConfig.Slot0.kI = config.kickerKI;
        kickerConfig.Slot0.kD = config.kickerKD;
        kickerConfig.Slot0.kS = config.kickerKS;
        kickerConfig.Slot0.kV = config.kickerKV;
        kickerConfig.Slot0.kA = config.kickerKA;
        kickerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        kickerConfig.ClosedLoopGeneral.ContinuousWrap = false;
        kickerConfig.Feedback.SensorToMechanismRatio = config.kickerMotorToOutputShaftRatio;

        kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerConfig.MotorOutput.Inverted = config.isKickerInverted ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kickerConfig.CurrentLimits.StatorCurrentLimit = config.kickerStatorCurrentLimit;

        kickerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.kickerPeakForwardTorqueCurrent;
        kickerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.kickerPeakReverseTorqueCurrent;

        kickerConfig.FutureProofConfigs = false;

        kickerMotor = new TalonFX(config.kickerCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));

        // Status signals
        kickerTorqueCurrent = kickerMotor.getTorqueCurrent().clone();
        kickerTemperature = kickerMotor.getDeviceTemp().clone();
        kickerVelocityStatusSignal = kickerMotor.getVelocity().clone();
        kickerMotorVoltage = kickerMotor.getMotorVoltage().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            kickerTorqueCurrent, kickerTemperature,
            kickerVelocityStatusSignal, kickerMotorVoltage);

        kickerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            kickerTorqueCurrent, kickerTemperature,
            kickerVelocityStatusSignal, kickerMotorVoltage);

        inputs.velocityRotationsPerSec = kickerVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.appliedVolts = kickerMotorVoltage.getValue().in(Volts);
        inputs.torqueCurrent = kickerTorqueCurrent.getValue().in(Amps);
        inputs.temperatureFahrenheit = kickerTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        kickerMotor.setControl(kickerMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setVoltage(double voltage) {
        kickerMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        kickerConfig.Slot0.kP = config.kP();
        kickerConfig.Slot0.kI = config.kI();
        kickerConfig.Slot0.kD = config.kD();
        kickerConfig.Slot0.kS = config.kS();
        kickerConfig.Slot0.kV = config.kV();
        kickerConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));
    }

    @Override
    public void enableEStop() {
        kickerConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));
    }

    @Override
    public void disableEStop() {
        kickerConfig.CurrentLimits.StatorCurrentLimit = config.kickerStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));
    }
}
