package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.KickerConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Kicker extends SubsystemBase {
    private static Kicker instance = null;

    public static Kicker getInstance() {
        if (instance == null) {
            instance = new Kicker();
        }
        return instance;
    }

    public enum KickerSetpoint {
        OFF(0.0),
        FEEDING(Double.NaN),
        KICKING(Double.NaN);

        private final double rps;
        KickerSetpoint(double rps) {
            this.rps = rps;
        }

        public double getRps() {
            return rps;
        }
    }

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

    private final KickerConfig config;

    private final DashboardMotorControlLoopConfigurator kickerControlLoopConfigurator;

    private double kickerSetpointRPS = 0.0;

    private Kicker() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.KICKER);
        config = ConfigLoader.load(
            "kicker",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.KICKER),
            KickerConfig.class
        );
        kickerIO = useSimulation ? new KickerIOSim(config) : new KickerIOTalonFX(config);

        kickerControlLoopConfigurator = new DashboardMotorControlLoopConfigurator("Kicker/kickerControlLoop",
            new DashboardMotorControlLoopConfigurator.MotorControlLoopConfig(
                config.kickerKP,
                config.kickerKI,
                config.kickerKD,
                config.kickerKS,
                config.kickerKV,
                config.kickerKA
            )
        );
    }

    @Override
    public void periodic() {
        kickerIO.updateInputs(kickerInputs);
        Logger.processInputs("Kicker", kickerInputs);

        // Handle control loop configuration updates
        if (kickerControlLoopConfigurator.hasChanged()) {
            kickerIO.configureControlLoop(kickerControlLoopConfigurator.getConfig());
        }

    }

    private void setKickerVelocity(double velocityRotationsPerSec) {
        kickerSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Kicker/velocitySetpointRPS", velocityRotationsPerSec);
        kickerIO.setVelocity(velocityRotationsPerSec);
    }

    public void setSetpoint(KickerSetpoint setpoint) {
        double targetRps = setpoint == KickerSetpoint.FEEDING
            ? config.feedingVelocityRPS
            : setpoint == KickerSetpoint.KICKING ? config.kickingVelocityRPS : setpoint.getRps();
        setKickerVelocity(targetRps);
    }

    // Mechanism getters
    public double getKickerVelocityRotationsPerSec() {
        return kickerInputs.velocityRotationsPerSec;
    }

    // Setpoint check methods
    @AutoLogOutput(key = "Kicker/isKickerAtSetpoint")
    public boolean isKickerAtSetpoint() {
        return Math.abs(kickerInputs.velocityRotationsPerSec - kickerSetpointRPS) < config.kickerVelocityToleranceRPS;
    }

}
