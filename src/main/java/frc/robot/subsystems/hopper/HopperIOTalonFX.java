package frc.robot.subsystems.hopper;

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
import frc.robot.configs.HopperConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final HopperConfig config;
    private final TalonFX hopperMotor;

    private final StatusSignal<AngularVelocity> hopperVelocityStatusSignal;
    private final StatusSignal<Voltage> hopperMotorVoltage;
    private final StatusSignal<Current> hopperTorqueCurrent;
    private final StatusSignal<Temperature> hopperTemperature;

    private final VelocityVoltage hopperMotorRequest = new VelocityVoltage(0).withSlot(0);

    private final TalonFXConfiguration hopperConfig;

    public HopperIOTalonFX(HopperConfig config) {
        this.config = config;
        hopperConfig = new TalonFXConfiguration();

        hopperConfig.Slot0.kP = config.hopperKP;
        hopperConfig.Slot0.kI = config.hopperKI;
        hopperConfig.Slot0.kD = config.hopperKD;
        hopperConfig.Slot0.kS = config.hopperKS;
        hopperConfig.Slot0.kV = config.hopperKV;
        hopperConfig.Slot0.kA = config.hopperKA;
        hopperConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        hopperConfig.ClosedLoopGeneral.ContinuousWrap = false;
        hopperConfig.Feedback.SensorToMechanismRatio = config.hopperMotorToOutputShaftRatio;

        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperConfig.MotorOutput.Inverted = config.isHopperInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.StatorCurrentLimit = config.hopperStatorCurrentLimit;

        hopperConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.hopperPeakForwardTorqueCurrent;
        hopperConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.hopperPeakReverseTorqueCurrent;

        hopperConfig.FutureProofConfigs = false;

        hopperMotor = new TalonFX(config.hopperCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(hopperConfig, 0.25));

        hopperTorqueCurrent = hopperMotor.getTorqueCurrent().clone();
        hopperTemperature = hopperMotor.getDeviceTemp().clone();
        hopperVelocityStatusSignal = hopperMotor.getVelocity().clone();
        hopperMotorVoltage = hopperMotor.getMotorVoltage().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            hopperTorqueCurrent, hopperTemperature,
            hopperVelocityStatusSignal, hopperMotorVoltage);

        hopperMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            hopperTorqueCurrent, hopperTemperature,
            hopperVelocityStatusSignal, hopperMotorVoltage);

        inputs.velocityRotationsPerSec = hopperVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.appliedVolts = hopperMotorVoltage.getValue().in(Volts);
        inputs.torqueCurrent = hopperTorqueCurrent.getValue().in(Amps);
        inputs.temperatureFahrenheit = hopperTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        hopperMotor.setControl(hopperMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setVoltage(double voltage) {
        hopperMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        hopperConfig.Slot0.kP = config.kP();
        hopperConfig.Slot0.kI = config.kI();
        hopperConfig.Slot0.kD = config.kD();
        hopperConfig.Slot0.kS = config.kS();
        hopperConfig.Slot0.kV = config.kV();
        hopperConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(hopperConfig, 0.25));
    }

    @Override
    public void enableEStop() {
        hopperConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(hopperConfig, 0.25));
    }

    @Override
    public void disableEStop() {
        hopperConfig.CurrentLimits.StatorCurrentLimit = config.hopperStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(hopperConfig, 0.25));
    }
}
