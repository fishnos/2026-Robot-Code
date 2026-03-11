package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.lib.util.DashboardMotorControlLoopConfigurator.MotorControlLoopConfig;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        // Hood
        public double hoodAngleRotations = 0;
        public double hoodVelocityRotationsPerSec = 0;
        public double hoodAppliedVolts = 0;
        public double hoodTorqueCurrent = 0;

        // Turret
        public double turretAngleRotations = 0;
        public double turretVelocityRotationsPerSec = 0;
        public double turretAppliedVolts = 0;
        public double turretTorqueCurrent = 0;

        // Flywheel (leader)
        public double flywheelVelocityRotationsPerSec = 0;
        public double flywheelAppliedVolts = 0;
        public double flywheelTorqueCurrent = 0;

        // Flywheel (follower)
        public double flywheelFollowerAppliedVolts = 0;
        public double flywheelFollowerTorqueCurrent = 0;

        // Temperatures
        public double hoodTemperatureFahrenheit = 0;
        public double turretTemperatureFahrenheit = 0;
        public double flywheelTemperatureFahrenheit = 0;
        public double flywheelFollowerTemperatureFahrenheit = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void setAngle(double angleRotations) {}
    public default void setTurretAngle(double angleRotations) {}
    public default void setShotVelocity(double velocityRotationsPerSec) {}
    public default void setHoodTorqueCurrentFOC(double torqueCurrentFOC) {}
    public default void setTurretTorqueCurrentFOC(double torqueCurrentFOC) {}
    public default void setFlywheelVoltage(double voltage) {}

    public default void configureHoodControlLoop(MotorControlLoopConfig config) {}
    public default void configureTurretControlLoop(MotorControlLoopConfig config) {}
    public default void configureFlywheelControlLoop(MotorControlLoopConfig config) {}
    public default void enableHoodEStop() {}
    public default void disableHoodEStop() {}
    public default void enableTurretEStop() {}
    public default void disableTurretEStop() {}
    public default void enableFlywheelEStop() {}
    public default void disableFlywheelEStop() {}
}
