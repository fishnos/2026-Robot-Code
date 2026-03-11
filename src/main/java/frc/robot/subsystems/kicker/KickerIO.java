package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface KickerIO {
    @AutoLog
    class KickerIOInputs {
        public double velocityRotationsPerSec = 0;
        public double appliedVolts = 0;
        public double torqueCurrent = 0;
        public double temperatureFahrenheit = 0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}
    public default void setVelocity(double velocityRotationsPerSec) {}
    public default void setVoltage(double voltage) {}
    public default void configureControlLoop(MotorControlLoopConfig config) {}
    public default void enableEStop() {}
    public default void disableEStop() {}
}
