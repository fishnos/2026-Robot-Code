package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;
import frc.robot.lib.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    static record TurretMotionMagicCommand(
        double positionRotations,
        double desiredVelocityRotPerSec,
        double commandedVelocityRotPerSec,
        double commandedAccelerationRotPerSec2
    ) {
        double motionMagicVelocityRotPerSec() {
            return Math.abs(commandedVelocityRotPerSec);
        }

        double motionMagicAccelerationRotPerSec2() {
            return Math.abs(commandedAccelerationRotPerSec2);
        }
    }

    private final TalonFX hoodMotor;
    private final TalonFX turretMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelFollowerMotor;

    private final StatusSignal<Angle> hoodPositionStatusSignal;
    private final StatusSignal<AngularVelocity> hoodVelocityStatusSignal;
    private final StatusSignal<Voltage> hoodMotorVoltage;

    private final StatusSignal<Angle> turretPositionStatusSignal;
    private final StatusSignal<AngularVelocity> turretVelocityStatusSignal;
    private final StatusSignal<Voltage> turretMotorVoltage;

    private final StatusSignal<AngularVelocity> flywheelVelocityStatusSignal;
    private final StatusSignal<Voltage> flywheelMotorVoltage;

    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodTemperature;

    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretTemperature;

    private final StatusSignal<Current> flywheelTorqueCurrent;
    private final StatusSignal<Temperature> flywheelTemperature;

    private final StatusSignal<Current> flywheelFollowerTorqueCurrent;
    private final StatusSignal<Temperature> flywheelFollowerTemperature;
    private final StatusSignal<Voltage> flywheelFollowerMotorVoltage;

    private final PositionVoltage hoodMotorRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    // private final DynamicMotionMagicVoltage turretMotorRequest =
    //     new DynamicMotionMagicVoltage(0, 0, 0).withSlot(0).withEnableFOC(true);
    private final PositionVoltage turretMotorRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    private final VelocityTorqueCurrentFOC flywheelMotorRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private final ShooterConfig config;
    private final TalonFXConfiguration hoodConfig;
    private final TalonFXConfiguration turretConfig;
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration flywheelFollowerConfig;
    private double previousTurretRequestedAngleRotations;
    private double previousTurretRequestedVelocityRotPerSec = 0.0;

    public ShooterIOTalonFX(ShooterConfig config) {
        this.config = config;

        // Hood motor configuration (positional control)
        hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = config.hoodKP;
        hoodConfig.Slot0.kI = config.hoodKI;
        hoodConfig.Slot0.kD = config.hoodKD;
        hoodConfig.Slot0.kS = config.hoodKS;
        hoodConfig.Slot0.kV = config.hoodKV;
        hoodConfig.Slot0.kA = config.hoodKA;
        hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        hoodConfig.ClosedLoopGeneral.ContinuousWrap = false;
        hoodConfig.Feedback.SensorToMechanismRatio = config.hoodMotorToOutputShaftRatio;

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = config.isHoodInverted ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = config.hoodStatorCurrentLimit;

        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.hoodPeakForwardTorqueCurrent;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.hoodPeakReverseTorqueCurrent;

        // Software limits for hood angle
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.hoodMaxAngleDegrees / 360.0;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.hoodMinAngleDegrees / 360.0;

        hoodConfig.FutureProofConfigs = false;

        hoodMotor = new TalonFX(config.hoodCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.setPosition(config.hoodStartingAngleDegrees / 360.0, 0.25));

        // Turret motor configuration (positional control, mirrors hood, Motion Magic)
        turretConfig = new TalonFXConfiguration();
        turretConfig.Slot0.kP = config.turretKP;
        turretConfig.Slot0.kI = config.turretKI;
        turretConfig.Slot0.kD = config.turretKD;
        turretConfig.Slot0.kS = config.turretKS;
        turretConfig.Slot0.kV = config.turretKV;
        turretConfig.Slot0.kA = config.turretKA;
        turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        turretConfig.ClosedLoopGeneral.ContinuousWrap = false;
        turretConfig.Feedback.SensorToMechanismRatio = config.turretMotorToOutputShaftRatio;

        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = config.isTurretInverted ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = config.turretStatorCurrentLimit;

        turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.turretPeakForwardTorqueCurrent;
        turretConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.turretPeakReverseTorqueCurrent;

        // Software limits for turret angle (convert degrees to rotations)
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.turretMaxAngleDeg / 360.0;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.turretMinAngleDeg / 360.0;

        // Motion Magic constraints (degrees/sec -> rotations/sec)
        double turretCruiseVelocityRotPerSec = config.turretMaxVelocityDegPerSec / 360.0;
        double turretAccelerationRotPerSec2 = config.turretMaxAccelerationDegPerSec2 / 360.0;
        double turretJerkRotPerSec3 = config.turretMaxJerkDegPerSec3 / 360.0;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = turretCruiseVelocityRotPerSec;
        turretConfig.MotionMagic.MotionMagicAcceleration = turretAccelerationRotPerSec2;
        turretConfig.MotionMagic.MotionMagicJerk = turretJerkRotPerSec3;

        turretConfig.FutureProofConfigs = false;

        turretMotor = new TalonFX(config.turretCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.setPosition(config.turretStartingAngleDeg / 360.0, 0.25));
        previousTurretRequestedAngleRotations = config.turretStartingAngleDeg / 360.0;

        // Flywheel motor configuration (velocity control) - Leader
        flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = config.flywheelKP;
        flywheelConfig.Slot0.kI = config.flywheelKI;
        flywheelConfig.Slot0.kD = config.flywheelKD;
        flywheelConfig.Slot0.kS = config.flywheelKS;
        flywheelConfig.Slot0.kV = config.flywheelKV;
        flywheelConfig.Slot0.kA = config.flywheelKA;
        
        flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        flywheelConfig.ClosedLoopGeneral.ContinuousWrap = false;
        flywheelConfig.Feedback.SensorToMechanismRatio = config.flywheelMotorToOutputShaftRatio;

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = config.isFlywheelInverted ?
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Current and torque limiting
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = config.flywheelStatorCurrentLimit;

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.flywheelPeakForwardTorqueCurrent;
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.flywheelPeakReverseTorqueCurrent;
        flywheelConfig.FutureProofConfigs = false;

        flywheelMotor = new TalonFX(config.flywheelCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

        // Flywheel follower motor configuration - uses same config as leader but set as follower
        flywheelFollowerConfig = new TalonFXConfiguration();

        flywheelFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // Follower inverted state will be handled by the Follower control request

        // Current and torque limiting (same as leader)
        flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = config.flywheelStatorCurrentLimit;

        flywheelFollowerConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.flywheelPeakForwardTorqueCurrent;
        flywheelFollowerConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.flywheelPeakReverseTorqueCurrent;

        flywheelFollowerConfig.FutureProofConfigs = false;

        flywheelFollowerMotor = new TalonFX(config.flywheelFollowerCanId, new CANBus(config.canBusName));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig, 0.25));

        // Set follower to follow the leader motor
        flywheelFollowerMotor.setControl(new Follower(config.flywheelCanId, 
            config.isFlywheelFollowerOppositeDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

        // Status signals
        hoodTorqueCurrent = hoodMotor.getTorqueCurrent().clone();
        hoodTemperature = hoodMotor.getDeviceTemp().clone();
        hoodMotorVoltage = hoodMotor.getMotorVoltage().clone();

        turretTorqueCurrent = turretMotor.getTorqueCurrent().clone();
        turretTemperature = turretMotor.getDeviceTemp().clone();
        turretMotorVoltage = turretMotor.getMotorVoltage().clone();

        flywheelTorqueCurrent = flywheelMotor.getTorqueCurrent().clone();
        flywheelTemperature = flywheelMotor.getDeviceTemp().clone();
        flywheelMotorVoltage = flywheelMotor.getMotorVoltage().clone();

        flywheelFollowerTorqueCurrent = flywheelFollowerMotor.getTorqueCurrent().clone();
        flywheelFollowerTemperature = flywheelFollowerMotor.getDeviceTemp().clone();
        flywheelFollowerMotorVoltage = flywheelFollowerMotor.getMotorVoltage().clone();

        hoodPositionStatusSignal = hoodMotor.getPosition().clone();
        hoodVelocityStatusSignal = hoodMotor.getVelocity().clone();

        turretPositionStatusSignal = turretMotor.getPosition().clone();
        turretVelocityStatusSignal = turretMotor.getVelocity().clone();

        flywheelVelocityStatusSignal = flywheelMotor.getVelocity().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            hoodTorqueCurrent, hoodTemperature, hoodMotorVoltage,
            turretTorqueCurrent, turretTemperature, turretMotorVoltage,
            flywheelTorqueCurrent, flywheelTemperature, flywheelMotorVoltage,
            flywheelFollowerTorqueCurrent, flywheelFollowerTemperature, flywheelFollowerMotorVoltage,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            turretPositionStatusSignal, turretVelocityStatusSignal,
            flywheelVelocityStatusSignal);

        hoodMotor.optimizeBusUtilization();
        turretMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        flywheelFollowerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            hoodTorqueCurrent, hoodTemperature, hoodMotorVoltage,
            turretTorqueCurrent, turretTemperature, turretMotorVoltage,
            flywheelTorqueCurrent, flywheelTemperature, flywheelMotorVoltage,
            flywheelFollowerTorqueCurrent, flywheelFollowerTemperature, flywheelFollowerMotorVoltage,
            hoodPositionStatusSignal, hoodVelocityStatusSignal,
            turretPositionStatusSignal, turretVelocityStatusSignal,
            flywheelVelocityStatusSignal);

        inputs.hoodAngleRotations = hoodPositionStatusSignal.getValue().in(Rotations);
        inputs.hoodVelocityRotationsPerSec = hoodVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.hoodAppliedVolts = hoodMotorVoltage.getValue().in(Volts);

        inputs.turretAngleRotations = turretPositionStatusSignal.getValue().in(Rotations);
        inputs.turretVelocityRotationsPerSec = turretVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.turretAppliedVolts = turretMotorVoltage.getValue().in(Volts);

        inputs.flywheelVelocityRotationsPerSec = flywheelVelocityStatusSignal.getValue().in(RotationsPerSecond);
        inputs.flywheelAppliedVolts = flywheelMotorVoltage.getValue().in(Volts);
        inputs.flywheelTorqueCurrent = flywheelTorqueCurrent.getValue().in(Amps);

        inputs.flywheelFollowerAppliedVolts = flywheelFollowerMotorVoltage.getValue().in(Volts);
        inputs.flywheelFollowerTorqueCurrent = flywheelFollowerTorqueCurrent.getValue().in(Amps);

        inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValue().in(Amps);
        inputs.hoodTemperatureFahrenheit = hoodTemperature.getValue().in(Fahrenheit);

        inputs.turretTorqueCurrent = turretTorqueCurrent.getValue().in(Amps);
        inputs.turretTemperatureFahrenheit = turretTemperature.getValue().in(Fahrenheit);

        inputs.flywheelTemperatureFahrenheit = flywheelTemperature.getValue().in(Fahrenheit);
        inputs.flywheelFollowerTemperatureFahrenheit = flywheelFollowerTemperature.getValue().in(Fahrenheit);

        Logger.recordOutput("Shooter/turretTalonSetpointRotations", turretMotor.getClosedLoopReference().getValueAsDouble());
    }

    @Override
    public void setAngle(double angleRotations) {
        // Clamp angle within software limits
        double minRot = config.hoodMinAngleDegrees / 360.0;
        double maxRot = config.hoodMaxAngleDegrees / 360.0;
        double clampedAngle = Math.max(minRot, Math.min(maxRot, angleRotations));
        hoodMotor.setControl(hoodMotorRequest.withPosition(clampedAngle));
    }

    @Override
    public void setTurretAngle(double angleRotations) {
        setTurretAngle(angleRotations, Double.NaN);
    }

    static TurretMotionMagicCommand calculateTurretMotionMagicCommand(
        double requestedAngleRotations,
        double requestedVelocityRotPerSec,
        double currentPositionRotations,
        double previousRequestedVelocityRotPerSec,
        double minRot,
        double maxRot,
        double maxVelocityRotPerSec,
        double maxAccelerationRotPerSec2,
        double loopPeriodSec
    ) {
        double clampedAngle = MathUtil.clamp(requestedAngleRotations, minRot, maxRot);

        double desiredVelocityRotPerSec;
        if (Double.isFinite(requestedVelocityRotPerSec)) {
            desiredVelocityRotPerSec = requestedVelocityRotPerSec;
        } else {
            // When no explicit tracking velocity is supplied, derive cruise velocity from the
            // remaining position error so a held setpoint does not collapse to zero cruise speed.
            double wrappedPositionErrorRotations = MathUtil.inputModulus(
                clampedAngle - currentPositionRotations,
                -0.5,
                0.5
            );
            desiredVelocityRotPerSec = wrappedPositionErrorRotations / loopPeriodSec;
        }

        double clampedDesiredVelocityRotPerSec = MathUtil.clamp(
            desiredVelocityRotPerSec,
            -maxVelocityRotPerSec,
            maxVelocityRotPerSec
        );
        double desiredAccelerationRotPerSec2 =
            (clampedDesiredVelocityRotPerSec - previousRequestedVelocityRotPerSec) / loopPeriodSec;
        double commandedAccelerationRotPerSec2 = MathUtil.clamp(
            desiredAccelerationRotPerSec2,
            -maxAccelerationRotPerSec2,
            maxAccelerationRotPerSec2
        );
        double commandedVelocityRotPerSec = MathUtil.clamp(
            previousRequestedVelocityRotPerSec + (commandedAccelerationRotPerSec2 * loopPeriodSec),
            -maxVelocityRotPerSec,
            maxVelocityRotPerSec
        );

        return new TurretMotionMagicCommand(
            clampedAngle,
            desiredVelocityRotPerSec,
            commandedVelocityRotPerSec,
            commandedAccelerationRotPerSec2
        );
    }

    @Override
    public void setTurretAngle(double angleRotations, double velocityRotationsPerSec) {
        double minRot = config.turretMinAngleDeg / 360.0;
        double maxRot = config.turretMaxAngleDeg / 360.0;
        double maxVelocityRotPerSec = turretConfig.MotionMagic.MotionMagicCruiseVelocity;
        double maxAccelerationRotPerSec2 = turretConfig.MotionMagic.MotionMagicAcceleration;
        TurretMotionMagicCommand motionMagicCommand = calculateTurretMotionMagicCommand(
            angleRotations,
            velocityRotationsPerSec,
            turretPositionStatusSignal.getValue().in(Rotations),
            previousTurretRequestedVelocityRotPerSec,
            minRot,
            maxRot,
            maxVelocityRotPerSec,
            maxAccelerationRotPerSec2,
            Constants.kLOOP_CYCLE_MS
        );
        double maxJerkRotPerSec3 = turretConfig.MotionMagic.MotionMagicJerk;

        turretMotor.setControl(
            turretMotorRequest
                .withPosition(motionMagicCommand.positionRotations())
                .withVelocity(motionMagicCommand.motionMagicVelocityRotPerSec())
                // .withAcceleration(motionMagicCommand.motionMagicAccelerationRotPerSec2())
                // .withJerk(Math.abs(maxJerkRotPerSec3))
        );
        Logger.recordOutput(
            "Shooter/turretCommandedVelocityRotationsPerSec",
            motionMagicCommand.commandedVelocityRotPerSec()
        );
        Logger.recordOutput(
            "Shooter/turretDesiredVelocityRotationsPerSec",
            motionMagicCommand.desiredVelocityRotPerSec()
        );
        Logger.recordOutput(
            "Shooter/turretCommandedAccelerationRotationsPerSec2",
            motionMagicCommand.commandedAccelerationRotPerSec2()
        );

        previousTurretRequestedAngleRotations = motionMagicCommand.positionRotations();
        previousTurretRequestedVelocityRotPerSec = motionMagicCommand.commandedVelocityRotPerSec();
    }

    @Override
    public void setShotVelocity(double velocityRotationsPerSec) {
        flywheelMotor.setControl(flywheelMotorRequest.withVelocity(velocityRotationsPerSec));
        // Follower automatically follows the leader
    }

    @Override
    public void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {
        hoodMotor.setControl(new TorqueCurrentFOC(torqueCurrentFOC));
    }

    @Override
    public void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {
        turretMotor.setControl(new TorqueCurrentFOC(torqueCurrentFOC));
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelMotor.setControl(new VoltageOut(voltage));
        // Follower automatically follows the leader
    }

    @Override
    public void configureHoodControlLoop(MotorControlLoopConfig config) {
        hoodConfig.Slot0.kP = config.kP();
        hoodConfig.Slot0.kI = config.kI();
        hoodConfig.Slot0.kD = config.kD();
        hoodConfig.Slot0.kS = config.kS();
        hoodConfig.Slot0.kV = config.kV();
        hoodConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void configureTurretControlLoop(MotorControlLoopConfig controlLoopConfig) {
        turretConfig.Slot0.kP = controlLoopConfig.kP();
        turretConfig.Slot0.kI = controlLoopConfig.kI();
        turretConfig.Slot0.kD = controlLoopConfig.kD();
        turretConfig.Slot0.kS = controlLoopConfig.kS();
        turretConfig.Slot0.kV = controlLoopConfig.kV();
        turretConfig.Slot0.kA = controlLoopConfig.kA();

        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
    }

    @Override
    public void configureFlywheelControlLoop(MotorControlLoopConfig config) {
        flywheelConfig.Slot0.kP = config.kP();
        flywheelConfig.Slot0.kI = config.kI();
        flywheelConfig.Slot0.kD = config.kD();
        flywheelConfig.Slot0.kS = config.kS();
        flywheelConfig.Slot0.kV = config.kV();
        flywheelConfig.Slot0.kA = config.kA();

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
    }

    @Override
    public void enableHoodEStop() {
        hoodConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void disableHoodEStop() {
        hoodConfig.CurrentLimits.StatorCurrentLimit = config.hoodStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void enableTurretEStop() {
        turretConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
    }

    @Override
    public void disableTurretEStop() {
        turretConfig.CurrentLimits.StatorCurrentLimit = config.turretStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretConfig, 0.25));
    }

    @Override
    public void enableFlywheelEStop() {
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 0;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 0;
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig, 0.25));
    }

    @Override
    public void disableFlywheelEStop() {
        flywheelConfig.CurrentLimits.StatorCurrentLimit = config.flywheelStatorCurrentLimit;
        flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = config.flywheelStatorCurrentLimit;
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig, 0.25));
    }
}
