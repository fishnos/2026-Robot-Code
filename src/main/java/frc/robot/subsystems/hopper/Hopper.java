package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.HopperConfig;
import frc.robot.constants.Constants;
import frc.robot.lib.util.ConfigLoader;
import frc.robot.lib.util.DashboardMotorControlLoopConfigurator;

public class Hopper extends SubsystemBase {
    private static Hopper instance = null;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public enum HopperSetpoint {
        OFF(0.0),
        FEEDING(Double.NaN),
        REVERSE(Double.NaN);

        private final double rps;
        HopperSetpoint(double rps) {
            this.rps = rps;
        }

        public double getRps() {
            return rps;
        }
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

    private final HopperConfig config;

    private final DashboardMotorControlLoopConfigurator hopperControlLoopConfigurator;

    private double hopperSetpointRPS = 0.0;

    private Hopper() {
        boolean useSimulation = Constants.shouldUseSimulation(Constants.SimOnlySubsystems.HOPPER);
        config = ConfigLoader.load(
            "hopper",
            ConfigLoader.getModeFolder(Constants.SimOnlySubsystems.HOPPER),
            HopperConfig.class
        );
        hopperIO = useSimulation ? new HopperIOSim(config) : new HopperIOTalonFX(config);

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

        if (hopperControlLoopConfigurator.hasChanged()) {
            hopperIO.configureControlLoop(hopperControlLoopConfigurator.getConfig());
        }

    }

    private void setHopperVelocity(double velocityRotationsPerSec) {
        hopperSetpointRPS = velocityRotationsPerSec;
        Logger.recordOutput("Hopper/velocitySetpointRPS", velocityRotationsPerSec);
        hopperIO.setVelocity(velocityRotationsPerSec);
    }

    public void setSetpoint(HopperSetpoint setpoint) {
        double targetRps = setpoint == HopperSetpoint.FEEDING
            ? config.feedingVelocityRPS
            : setpoint == HopperSetpoint.REVERSE ? config.reverseVelocityRPS : setpoint.getRps();
        setHopperVelocity(targetRps);
    }

    public double getHopperVelocityRotationsPerSec() {
        return hopperInputs.velocityRotationsPerSec;
    }

    @AutoLogOutput(key = "Hopper/isHopperAtSetpoint")
    public boolean isHopperAtSetpoint() {
        return Math.abs(hopperInputs.velocityRotationsPerSec - hopperSetpointRPS) < config.hopperVelocityToleranceRPS;
    }
}
