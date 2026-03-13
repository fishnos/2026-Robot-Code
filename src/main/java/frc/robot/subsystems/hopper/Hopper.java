package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.HopperConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Hopper extends SubsystemBase {
    private static final double HOPPER_VOLTAGE_SETPOINT_TOLERANCE_VOLTS = 0.25;

    private enum HopperControlMode {
        VELOCITY,
        VOLTAGE
    }

    private static Hopper instance = null;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public enum HopperSetpoint {
        OFF,
        FEEDING,
        FEEDING_IDLE,
        REVERSE
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

    private final HopperConfig config;

    private final DashboardMotorControlLoopConfigurator hopperControlLoopConfigurator;
    private boolean pendingHopperControlLoopConfigApply = false;
    private final boolean enableConnectionAlerts;
    private final Alert hopperDisconnectedAlert;

    private HopperControlMode hopperControlMode = HopperControlMode.VELOCITY;
    private double hopperSetpointRPS = 0.0;
    private double hopperSetpointVolts = 0.0;

    private Hopper() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.HOPPER);
        config = ConfigLoader.load(
            "hopper",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.HOPPER),
            HopperConfig.class
        );
        hopperIO = useSimulation ? new HopperIOSim(config) : new HopperIOTalonFX(config);
        enableConnectionAlerts = !useSimulation && Constants.currentMode != Constants.Mode.REPLAY;
        hopperDisconnectedAlert = new Alert("Hopper motor is disconnected.", AlertType.kWarning);

        hopperControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Hopper/hopperControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.hopperKP,
                config.hopperKI,
                config.hopperKD,
                config.hopperKS,
                config.hopperKV,
                config.hopperKA
            )
        );
    }

    @Override
    public void periodic() {
        hopperIO.updateInputs(hopperInputs);
        Logger.processInputs("Hopper", hopperInputs);

        hopperDisconnectedAlert.set(enableConnectionAlerts && !hopperInputs.hopperMotorConnected);

        pendingHopperControlLoopConfigApply |= hopperControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled() && pendingHopperControlLoopConfigApply) {
            hopperIO.configureControlLoop(hopperControlLoopConfigurator.getConfig());
            pendingHopperControlLoopConfigApply = false;
        }
    }

    public void setHopperVelocity(double velocityRotationsPerSec) {
        hopperControlMode = HopperControlMode.VELOCITY;
        hopperSetpointRPS = velocityRotationsPerSec;
        hopperSetpointVolts = Double.NaN;
        Logger.recordOutput("Hopper/controlMode", hopperControlMode.name());
        Logger.recordOutput("Hopper/velocitySetpointRPS", velocityRotationsPerSec);
        Logger.recordOutput("Hopper/voltageSetpointVolts", Double.NaN);
        hopperIO.setVelocity(velocityRotationsPerSec);
    }

    public void setHopperVoltage(double voltage) {
        hopperControlMode = HopperControlMode.VOLTAGE;
        hopperSetpointRPS = Double.NaN;
        hopperSetpointVolts = voltage;
        Logger.recordOutput("Hopper/controlMode", hopperControlMode.name());
        Logger.recordOutput("Hopper/velocitySetpointRPS", Double.NaN);
        Logger.recordOutput("Hopper/voltageSetpointVolts", voltage);
        hopperIO.setVoltage(voltage);
    }

    public void setSetpoint(HopperSetpoint setpoint) {
        switch (setpoint) {
            case OFF:
                setHopperVelocity(0.0);
                break;
            case FEEDING:
                setHopperVelocity(config.feedingVelocityRPS);
                break;
            case FEEDING_IDLE:
                setHopperVoltage(config.feedingIdleVoltage);
                break;
            case REVERSE:
                setHopperVelocity(config.reverseVelocityRPS);
                break;
        }
    }

    public double getHopperVelocityRotationsPerSec() {
        return hopperInputs.velocityRotationsPerSec;
    }

    public void enableEStop() {
        hopperIO.enableEStop();
    }

    public void disableEStop() {
        hopperIO.disableEStop();
    }

    @AutoLogOutput(key = "Hopper/isHopperAtSetpoint")
    public boolean isHopperAtSetpoint() {
        if (!hopperInputs.hopperMotorConnected) {
            return false;
        }

        return hopperControlMode == HopperControlMode.VELOCITY
            ? Math.abs(hopperInputs.velocityRotationsPerSec - hopperSetpointRPS) < config.hopperVelocityToleranceRPS
            : Math.abs(hopperInputs.appliedVolts - hopperSetpointVolts) < HOPPER_VOLTAGE_SETPOINT_TOLERANCE_VOLTS;
    }

    @AutoLogOutput(key = "Hopper/isHopperMotorConnected")
    public boolean isHopperMotorConnected() {
        return hopperInputs.hopperMotorConnected;
    }
}
