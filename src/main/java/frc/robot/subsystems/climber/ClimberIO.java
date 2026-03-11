package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double positionRotations = 0;
        public double velocityRotationsPerSec = 0;
        public double appliedVolts = 0;
        public double torqueCurrent = 0;
        public double temperatureFahrenheit = 0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setPosition(double positionRotations) {}
    public default void setVoltage(double voltage) {}
    public default void configureControlLoop(MotorControlLoopConfig config) {}
    public default void enableEStop() {}
    public default void disableEStop() {}
}
