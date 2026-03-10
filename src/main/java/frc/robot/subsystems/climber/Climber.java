package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.ClimberConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Climber extends SubsystemBase {
    private static Climber instance = null;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public enum DesiredState {
        DISABLED,
        RETRACTED,
        EXTENDED,
        CLIMBED
    }

    public enum CurrentState {
        DISABLED,
        RETRACTED,
        EXTENDED,
        CLIMBED
    }

    public enum ClimberSetpoint {
        RETRACTED {
            @Override
            public double positionRotations(ClimberConfig config) {
                return config.climberHomePositionRotations;
            }
        },
        EXTENDED {
            @Override
            public double positionRotations(ClimberConfig config) {
                return config.climberReadyPositionRotations;
            }
        },
        CLIMBED {
            @Override
            public double positionRotations(ClimberConfig config) {
                return config.climberLiftedPositionRotations;
            }

            @Override
            public double maxVelocityRotationsPerSec(ClimberConfig config) {
                return config.climberClimbingProfileMaxVelocityRotationsPerSec;
            }

            @Override
            public double maxAccelerationRotationsPerSec2(ClimberConfig config) {
                return config.climberClimbingProfileMaxAccelerationRotationsPerSec2;
            }
        };

        public abstract double positionRotations(ClimberConfig config);

        public double maxVelocityRotationsPerSec(ClimberConfig config) {
            return config.climberDefaultProfileMaxVelocityRotationsPerSec;
        }

        public double maxAccelerationRotationsPerSec2(ClimberConfig config) {
            return config.climberDefaultProfileMaxAccelerationRotationsPerSec2;
        }
    }

    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
    private final ClimberConfig config;
    private final DashboardMotorControlLoopConfigurator climberControlLoopConfigurator;

    private DesiredState desiredState = DesiredState.DISABLED;
    private CurrentState currentState = CurrentState.DISABLED;
    private DesiredState activeState = DesiredState.DISABLED;
    private CurrentState lastAchievedState;
    private boolean pendingClimberControlLoopConfigApply = false;
    private double targetPositionRotations = 0.0;
    private double targetMaxVelocityRotationsPerSec = 0.0;
    private double targetMaxAccelerationRotationsPerSec2 = 0.0;
    private final boolean enableConnectionAlerts;
    private final Alert climberDisconnectedAlert;

    private Climber() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.CLIMBER);
        config = ConfigLoader.load(
            "climber",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.CLIMBER),
            ClimberConfig.class
        );
        climberIO = useSimulation ? new ClimberIOSim(config) : new ClimberIOTalonFX(config);
        enableConnectionAlerts = !useSimulation && Constants.currentMode != Constants.Mode.REPLAY;
        climberDisconnectedAlert = new Alert("Climber motor is disconnected.", AlertType.kWarning);

        climberControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Climber/climberControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.climberKP,
                config.climberKI,
                config.climberKD,
                config.climberKS,
                config.climberKV,
                config.climberKA
            )
        );
        lastAchievedState = resolveAchievedState(config.climberStartingPositionRotations, CurrentState.RETRACTED);
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);

        climberDisconnectedAlert.set(enableConnectionAlerts && !climberInputs.climberMotorConnected);

        handleStateTransitions();

        handleCurrentState();

        pendingClimberControlLoopConfigApply |= climberControlLoopConfigurator.hasChanged();
        if (DriverStation.isDisabled() && pendingClimberControlLoopConfigApply) {
            climberIO.configureControlLoop(climberControlLoopConfigurator.getConfig());
            pendingClimberControlLoopConfigApply = false;
        }
    }

    private void handleStateTransitions() {
        lastAchievedState = resolveAchievedState(climberInputs.positionRotations, lastAchievedState);

        switch (desiredState) {
            case DISABLED:
                currentState = CurrentState.DISABLED;
                activeState = DesiredState.DISABLED;
                break;
            case RETRACTED, EXTENDED, CLIMBED:
                currentState = lastAchievedState;
                activeState = resolveActiveState(currentState, desiredState);
                break;
        }
    }

    static DesiredState resolveActiveState(CurrentState currentState, DesiredState desiredState) {
        if (desiredState == DesiredState.DISABLED) {
            return DesiredState.DISABLED;
        }

        if (desiredState == DesiredState.CLIMBED
            && currentState != CurrentState.EXTENDED
            && currentState != CurrentState.CLIMBED) {
            return DesiredState.EXTENDED;
        }

        if (currentState == CurrentState.CLIMBED && desiredState != DesiredState.CLIMBED) {
            return DesiredState.EXTENDED;
        }

        return desiredState;
    }

    private CurrentState resolveAchievedState(double positionRotations, CurrentState fallbackState) {
        if (isWithinTolerance(positionRotations, config.climberHomePositionRotations)) {
            return CurrentState.RETRACTED;
        }
        if (isWithinTolerance(positionRotations, config.climberReadyPositionRotations)) {
            return CurrentState.EXTENDED;
        }
        if (isWithinTolerance(positionRotations, config.climberLiftedPositionRotations)) {
            return CurrentState.CLIMBED;
        }
        return fallbackState;
    }

    private boolean isWithinTolerance(double positionRotations, double targetPositionRotations) {
        return Math.abs(positionRotations - targetPositionRotations) < config.climberPositionToleranceRotations;
    }

    private void handleCurrentState() {
        switch (activeState) {
            case DISABLED:
                setDisabled();
                break;
            case RETRACTED:
                setClimberSetpoint(ClimberSetpoint.RETRACTED);
                break;
            case EXTENDED:
                setClimberSetpoint(ClimberSetpoint.EXTENDED);
                break;
            case CLIMBED:
                setClimberSetpoint(ClimberSetpoint.CLIMBED);
                break;
        }
    }

    private void setClimberSetpoint(ClimberSetpoint setpoint) {
        targetPositionRotations = setpoint.positionRotations(config);
        targetMaxVelocityRotationsPerSec = setpoint.maxVelocityRotationsPerSec(config);
        targetMaxAccelerationRotationsPerSec2 = setpoint.maxAccelerationRotationsPerSec2(config);
        Logger.recordOutput("Climber/setpoint", setpoint.toString());
        Logger.recordOutput("Climber/positionSetpointRotations", targetPositionRotations);
        Logger.recordOutput("Climber/maxVelocitySetpointRotationsPerSec", targetMaxVelocityRotationsPerSec);
        Logger.recordOutput("Climber/maxAccelerationSetpointRotationsPerSec2", targetMaxAccelerationRotationsPerSec2);
        climberIO.setPosition(
            targetPositionRotations,
            targetMaxVelocityRotationsPerSec,
            targetMaxAccelerationRotationsPerSec2
        );
    }

    private boolean isAtPosition(double positionRotations) {
        return Math.abs(climberInputs.positionRotations - positionRotations) < config.climberPositionToleranceRotations;
    }

    public void setDesiredState(DesiredState desiredState) {
        this.desiredState = desiredState;
    }

    public void setSetpoint(ClimberSetpoint setpoint) {
        switch (setpoint) {
            case RETRACTED:
                setDesiredState(DesiredState.RETRACTED);
                break;
            case EXTENDED:
                setDesiredState(DesiredState.EXTENDED);
                break;
            case CLIMBED:
                setDesiredState(DesiredState.CLIMBED);
                break;
        }
    }

    public void setDisabled() {
        targetPositionRotations = climberInputs.positionRotations;
        targetMaxVelocityRotationsPerSec = 0.0;
        targetMaxAccelerationRotationsPerSec2 = 0.0;
        climberIO.setVoltage(0);
    }

    public void enableEStop() {
        climberIO.enableEStop();
    }

    public void disableEStop() {
        climberIO.disableEStop();
    }

    public double getClimberPositionRotations() {
        return climberInputs.positionRotations;
    }

    @AutoLogOutput(key = "Climber/desiredState")
    public DesiredState getDesiredState() {
        return desiredState;
    }

    @AutoLogOutput(key = "Climber/currentState")
    public CurrentState getCurrentState() {
        return currentState;
    }

    @AutoLogOutput(key = "Climber/activeState")
    public DesiredState getActiveState() {
        return activeState;
    }

    @AutoLogOutput(key = "Climber/lastAchievedState")
    public CurrentState getLastAchievedState() {
        return lastAchievedState;
    }

    @AutoLogOutput(key = "Climber/isAtSetpoint")
    public boolean isAtSetpoint() {
        return climberInputs.climberMotorConnected && isAtPosition(targetPositionRotations);
    }

    @AutoLogOutput(key = "Climber/targetMaxVelocityRotationsPerSec")
    public double getTargetMaxVelocityRotationsPerSec() {
        return targetMaxVelocityRotationsPerSec;
    }

    @AutoLogOutput(key = "Climber/targetMaxAccelerationRotationsPerSec2")
    public double getTargetMaxAccelerationRotationsPerSec2() {
        return targetMaxAccelerationRotationsPerSec2;
    }

    @AutoLogOutput(key = "Climber/isAtDesiredState")
    public boolean isAtDesiredState() {
        return switch (desiredState) {
            case DISABLED -> currentState == CurrentState.DISABLED;
            case RETRACTED -> currentState == CurrentState.RETRACTED;
            case EXTENDED -> currentState == CurrentState.EXTENDED;
            case CLIMBED -> currentState == CurrentState.CLIMBED;
        };
    }

    @AutoLogOutput(key = "Climber/isRetracted")
    public boolean isRetracted() {
        return climberInputs.climberMotorConnected && currentState == CurrentState.RETRACTED;
    }

    @AutoLogOutput(key = "Climber/isExtended")
    public boolean isExtended() {
        return climberInputs.climberMotorConnected && currentState == CurrentState.EXTENDED;
    }

    @AutoLogOutput(key = "Climber/isClimbed")
    public boolean isClimbed() {
        return climberInputs.climberMotorConnected && currentState == CurrentState.CLIMBED;
    }

    @AutoLogOutput(key = "Climber/isClimberMotorConnected")
    public boolean isClimberMotorConnected() {
        return climberInputs.climberMotorConnected;
    }
}
