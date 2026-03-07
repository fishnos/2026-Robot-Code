package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.configs.IntakeConfig;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeMotor;

    private final StatusSignal<AngularVelocity> intakeVelocityStatusSignal;
    private final StatusSignal<Current> intakeTorqueCurrent;
    private final StatusSignal<Temperature> intakeTemperature;

    private final VelocityVoltage intakeMotorRequest = new VelocityVoltage(0).withSlot(0);

    private final TalonFXConfiguration intakeConfig;

    public IntakeIOTalonFX(IntakeConfig config) {
        intakeConfig = new TalonFXConfiguration();

        intakeConfig.Slot0.kP = config.intakeKP;
        intakeConfig.Slot0.kI = config.intakeKI;
        intakeConfig.Slot0.kD = config.intakeKD;
        intakeConfig.Slot0.kS = config.intakeKS;
        intakeConfig.Slot0.kV = config.intakeKV;
        intakeConfig.Slot0.kA = config.intakeKA;
        intakeConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        intakeConfig.ClosedLoopGeneral.ContinuousWrap = false;
        intakeConfig.Feedback.SensorToMechanismRatio = config.intakeMotorToOutputShaftRatio;

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = config.isIntakeInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = config.intakeSupplyCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = config.intakeSupplyCurrentLimitLowerLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = config.intakeSupplyCurrentLimitLowerTime;

        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = config.intakeStatorCurrentLimit;

        intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.intakePeakForwardTorqueCurrent;
        intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.intakePeakReverseTorqueCurrent;

        intakeConfig.FutureProofConfigs = false;

        intakeMotor = new TalonFX(config.intakeCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

        intakeTorqueCurrent = intakeMotor.getTorqueCurrent().clone();
        intakeTemperature = intakeMotor.getDeviceTemp().clone();
        intakeVelocityStatusSignal = intakeMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
                intakeTorqueCurrent, intakeTemperature,
                intakeVelocityStatusSignal);

        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                intakeTorqueCurrent, intakeTemperature,
                intakeVelocityStatusSignal);

        inputs.velocityRotationsPerSec = intakeVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.appliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.torqueCurrent = intakeTorqueCurrent.getValue().in(Amps);
        inputs.temperatureFahrenheit = intakeTemperature.getValue().in(Fahrenheit);
    }

    @Override
    public void setVelocity(double velocityRotationsPerSec) {
        intakeMotor.setControl(intakeMotorRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void configureControlLoop(MotorControlLoopConfig config) {
        intakeConfig.Slot0.kP = config.kP();
        intakeConfig.Slot0.kI = config.kI();
        intakeConfig.Slot0.kD = config.kD();
        intakeConfig.Slot0.kS = config.kS();
        intakeConfig.Slot0.kV = config.kV();
        intakeConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));
    }
}
