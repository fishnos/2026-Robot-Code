package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShooterTurretResolutionTest {
    private static final double EPS = 1e-9;

    @Test
    // When multiple wrapped equivalents exist, choose the one closest to current turret angle.
    void resolveTurretTargetDegrees_selectsNearestWrappedEquivalent() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            10.0,
            350.0,
            -720.0,
            720.0
        );

        assertFalse(resolution.usedUnwindFallback());
        assertEquals(370.0, resolution.targetAngleDeg(), EPS);
    }

    @Test
    // If only one wrapped equivalent is inside the range, it should be selected directly.
    void resolveTurretTargetDegrees_handlesSingleWrappedEquivalentInsideRange() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            190.0,
            -150.0,
            -180.0,
            180.0
        );

        assertFalse(resolution.usedUnwindFallback());
        assertEquals(-170.0, resolution.targetAngleDeg(), EPS);
    }

    @Test
    // If no wrapped angle is legal, fallback should unwind toward the bound nearest zero.
    void resolveTurretTargetDegrees_usesUnwindFallbackTowardZeroWhenNoWrappedEquivalent() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            0.0,
            0.0,
            100.0,
            200.0
        );

        assertTrue(resolution.usedUnwindFallback());
        assertEquals(100.0, resolution.targetAngleDeg(), EPS);
        assertEquals(100.0, resolution.unwindTargetDeg(), EPS);
    }

    @Test
    // Negative-only range fallback should choose the upper (closer-to-zero) boundary.
    void resolveTurretTargetDegrees_unwindFallbackChoosesBoundClosestToZeroForNegativeRange() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            0.0,
            0.0,
            -200.0,
            -100.0
        );

        assertTrue(resolution.usedUnwindFallback());
        assertEquals(-100.0, resolution.targetAngleDeg(), EPS);
        assertEquals(-100.0, resolution.unwindTargetDeg(), EPS);
    }

    @Test
    // Near a soft limit, avoid large wrapped jumps by clamping the current branch when it is closer.
    void resolveTurretTargetDegrees_prefersClampedCurrentBranchWhenMuchCloserThanWrappedCandidate() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            -94.0,
            -447.0,
            -450.0,
            630.0
        );

        assertFalse(resolution.usedUnwindFallback());
        assertEquals(-450.0, resolution.targetAngleDeg(), EPS);
    }
}
