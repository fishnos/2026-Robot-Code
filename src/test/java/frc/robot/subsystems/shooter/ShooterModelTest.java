package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterModelTest {
    private static final double EPS = 1e-9;

    @Test
    // Exit velocity should be the average of flywheel and back-roller surface velocities.
    void calculateShotExitVelocityMetersPerSec_matchesWheelSurfaceAverage() {
        double rps = 50.0;
        double flywheelRadius = 0.05;
        double backRollerGearRatio = 0.6;
        double backRollerRadius = 0.03;

        double expectedFlywheelSurfaceVel = rps * 2.0 * Math.PI * flywheelRadius;
        double expectedBackRollerSurfaceVel = rps * backRollerGearRatio * 2.0 * Math.PI * backRollerRadius;
        double expected = (expectedFlywheelSurfaceVel + expectedBackRollerSurfaceVel) / 2.0;

        double actual = Shooter.calculateShotExitVelocityMetersPerSec(
            rps,
            flywheelRadius,
            backRollerGearRatio,
            backRollerRadius
        );

        assertEquals(expected, actual, EPS);
    }

    @Test
    // Backspin model should match differential surface velocity converted to RPM.
    void calculateBackSpinRPM_matchesDifferentialSurfaceVelocityModel() {
        double rps = 40.0;
        double flywheelRadius = 0.05;
        double backRollerGearRatio = 0.5;
        double backRollerRadius = 0.02;
        double ballRadius = 0.12;

        double flywheelSurfaceVel = rps * 2.0 * Math.PI * flywheelRadius;
        double backRollerSurfaceVel = rps * backRollerGearRatio * 2.0 * Math.PI * backRollerRadius;
        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        double expected = (deltaV / ballRadius) * 60.0 / (2.0 * Math.PI);

        double actual = Shooter.calculateBackSpinRPM(
            rps,
            flywheelRadius,
            backRollerGearRatio,
            backRollerRadius,
            ballRadius
        );

        assertEquals(expected, actual, EPS);
    }

    @Test
    // Zero wheel speed should produce zero shot exit velocity.
    void calculateShotExitVelocityMetersPerSec_zeroFlywheelSpeed_returnsZero() {
        double actual = Shooter.calculateShotExitVelocityMetersPerSec(
            0.0,
            0.05,
            0.6,
            0.03
        );

        assertEquals(0.0, actual, EPS);
    }

    @Test
    // Geometry where back roller is effectively faster should produce negative spin (topspin).
    void calculateBackSpinRPM_canReturnNegativeForTopspinConfiguration() {
        double actual = Shooter.calculateBackSpinRPM(
            30.0,
            0.03,
            1.5,
            0.06,
            0.12
        );

        assertTrue(actual < 0.0);
    }
}
