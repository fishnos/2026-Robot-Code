package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.IntakeConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public enum IntakeSetpoint {
        DISABLED,
        STOWED,
        DEPLOYED,
        INTAKING,
        OUTTAKING
    }

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private final IntakeConfig config;

    private final DashboardMotorControlLoopConfigurator rollerControlLoopConfigurator;
    private final DashboardMotorControlLoopConfigurator pivotControlLoopConfigurator;

    private double rollerSetpointRPS = 0.0;
    private double pivotSetpointRotations = 0.0;

    private Intake() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.INTAKE);
        config = ConfigLoader.load(
            "intake",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.INTAKE),
            IntakeConfig.class
        );
        intakeIO = useSimulation ? new IntakeIOSim(config) : new IntakeIOTalonFX(config);

        rollerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Intake/rollerControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.rollerKP,
                config.rollerKI,
                config.rollerKD,
                config.rollerKS,
                config.rollerKV,
                config.rollerKA
            )
        );
        pivotControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Intake/pivotControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.pivotKP,
                config.pivotKI,
                config.pivotKD,
                config.pivotKS,
                config.pivotKV,
                config.pivotKA,
                config.pivotKG
            )
        );
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);

        if (rollerControlLoopConfigurator.hasChanged()) {
            intakeIO.configureControlLoop(rollerControlLoopConfigurator.getConfig());
        }
        if (pivotControlLoopConfigurator.hasChanged()) {
            intakeIO.configurePivotControlLoop(pivotControlLoopConfigurator.getConfig());
        }
    }

    private void setRollerVelocity(double velocityRotationsPerSec) {
        rollerSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Intake/rollerVelocitySetpointRPS", velocityRotationsPerSec);
        intakeIO.setVelocity(velocityRotationsPerSec);
    }

    private void setPivotAngle(double angleRotations) {
        pivotSetpointRotations = angleRotations;
        Logger.recordOutput("Intake/pivotAngleSetpointRotations", angleRotations);
        intakeIO.setPivotAngle(angleRotations);
    }

    public void setSetpoint(IntakeSetpoint setpoint) {
        switch (setpoint) {
            case DISABLED:
                setPivotAngle(intakeInputs.pivotAngleRotations);
                setRollerVelocity(0.0);
                break;
            case STOWED:
                setPivotAngle(config.pivotUpAngleRotations);
                setRollerVelocity(0.0);
                break;
            case DEPLOYED:
                setPivotAngle(config.pivotDownAngleRotations);
                setRollerVelocity(0.0);
                break;
            case INTAKING:
                setPivotAngle(config.pivotDownAngleRotations);
                setRollerVelocity(config.intakingVelocityRPS);
                break;
            case OUTTAKING:
                setPivotAngle(config.pivotDownAngleRotations);
                setRollerVelocity(config.outtakingVelocityRPS);
                break;
        }
    }

    public double getRollerVelocityRotationsPerSec() {
        return intakeInputs.rollerVelocityRotationsPerSec;
    }

    public double getPivotAngleRotations() {
        return intakeInputs.pivotAngleRotations;
    }

    public void enableRollerEStop() {
        intakeIO.enableRollerEStop();
    }

    public void disableRollerEStop() {
        intakeIO.disableRollerEStop();
    }

    public void enablePivotEStop() {
        intakeIO.enablePivotEStop();
    }

    public void disablePivotEStop() {
        intakeIO.disablePivotEStop();
    }

    @AutoLogOutput(key = "Intake/isRollerAtSetpoint")
    public boolean isRollerAtSetpoint() {
        return Math.abs(intakeInputs.rollerVelocityRotationsPerSec - rollerSetpointRPS)
            < config.rollerVelocityToleranceRPS;
    }

    @AutoLogOutput(key = "Intake/isPivotAtSetpoint")
    public boolean isPivotAtSetpoint() {
        return Math.abs(intakeInputs.pivotAngleRotations - pivotSetpointRotations)
            < config.pivotAngleToleranceRotations;
    }

    @AutoLogOutput(key = "Intake/isStowed")
    public boolean isStowed() {
        return Math.abs(intakeInputs.pivotAngleRotations - config.pivotUpAngleRotations)
            < config.pivotAngleToleranceRotations;
    }
}
