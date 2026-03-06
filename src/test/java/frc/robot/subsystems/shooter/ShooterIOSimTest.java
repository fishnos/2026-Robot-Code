package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.junit.jupiter.api.Test;

class ShooterIOSimTest {
    @Test
    void calculateTurretControlVoltage_preservesBackwardTrackingDirection() {
        double controlVoltage = ShooterIOSim.calculateTurretControlVoltage(
            0.50,
            0.30,
            -1.0,
            new PIDController(40.0, 0.0, 0.0),
            new SimpleMotorFeedforward(0.0, 0.0, 0.0)
        );

        assertTrue(controlVoltage < 0.0);
    }
}
