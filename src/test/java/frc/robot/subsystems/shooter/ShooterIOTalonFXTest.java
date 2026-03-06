package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShooterIOTalonFXTest {
    private static final double EPS = 1e-9;

    @Test
    void calculateTurretPositionCommand_preservesSignedVelocity() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            0.2,
            -1.0,
            0.0,
            1.0
        );

        assertEquals(0.2, command.positionRotations(), EPS);
        assertEquals(-1.0, command.velocityRotPerSec(), EPS);
    }

    @Test
    void calculateTurretPositionCommand_clampsRequestedPositionInsideSoftLimits() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            1.4,
            0.5,
            0.0,
            1.0
        );

        assertEquals(1.0, command.positionRotations(), EPS);
        assertEquals(0.5, command.velocityRotPerSec(), EPS);
    }

    @Test
    void calculateTurretPositionCommand_treatsNonFiniteVelocityAsZero() {
        ShooterIOTalonFX.TurretPositionCommand command = ShooterIOTalonFX.calculateTurretPositionCommand(
            0.4,
            Double.NaN,
            0.0,
            1.0
        );

        assertEquals(0.4, command.positionRotations(), EPS);
        assertEquals(0.0, command.velocityRotPerSec(), EPS);
    }
}
