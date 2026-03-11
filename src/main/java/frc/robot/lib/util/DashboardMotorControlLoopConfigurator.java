package frc.robot.lib.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for configuring the motor control loop parameters on the dashboard. 
 * Publishes Logged Network Numbers for the motor control loop parameters.
 * 
 */
public class DashboardMotorControlLoopConfigurator {
    public record MotorControlLoopConfig(
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG
    ) {
        public MotorControlLoopConfig(
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA
        ) {
            this(kP, kI, kD, kS, kV, kA, 0.0);
        }
    }

    private final MotorControlLoopConfig defaults;
    private MotorControlLoopConfig previousConfigSinceHsaChangedCall;
    private final LoggedNetworkNumber[] numbers;

    public DashboardMotorControlLoopConfigurator(String tableKey, MotorControlLoopConfig defaults) {
        // make LoggedNetworkNumbers for the config
        this.defaults = defaults;
        this.previousConfigSinceHsaChangedCall = defaults;
        this.numbers = new LoggedNetworkNumber[7];
    
        this.numbers[0] = new LoggedNetworkNumber(tableKey + "/kP", defaults.kP);
        this.numbers[1] = new LoggedNetworkNumber(tableKey + "/kI", defaults.kI);
        this.numbers[2] = new LoggedNetworkNumber(tableKey + "/kD", defaults.kD);
        this.numbers[3] = new LoggedNetworkNumber(tableKey + "/kS", defaults.kS);
        this.numbers[4] = new LoggedNetworkNumber(tableKey + "/kV", defaults.kV);
        this.numbers[5] = new LoggedNetworkNumber(tableKey + "/kA", defaults.kA);
        this.numbers[6] = new LoggedNetworkNumber(tableKey + "/kG", defaults.kG);
    }

    public MotorControlLoopConfig getConfig() {
        return new MotorControlLoopConfig(
            numbers[0].get(),
            numbers[1].get(),
            numbers[2].get(),
            numbers[3].get(),
            numbers[4].get(),
            numbers[5].get(),
            numbers[6].get()
        );
    }

    public boolean hasChanged() {
        // compare the default to the current dashboard values and return true if any of them have changed
        MotorControlLoopConfig currentConfig = getConfig();
        if (currentConfig.equals(previousConfigSinceHsaChangedCall)) {
            return false;
        }
        previousConfigSinceHsaChangedCall = currentConfig;
        return true;
    }
}
