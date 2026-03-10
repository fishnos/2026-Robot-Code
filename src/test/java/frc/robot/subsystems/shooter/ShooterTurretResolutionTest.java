package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.junit.jupiter.api.Test;

class ShooterTurretResolutionTest {
    private static final double EPS = 1e-9;

    @Test
    // When multiple wrapped equivalents exist, choose the one closest to the previous mechanism goal.
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
    // If no wrapped angle is legal, fallback should choose the limit with lower aim error.
    void resolveTurretTargetDegrees_usesUnwindFallbackTowardNearestAimBoundWhenNoWrappedEquivalent() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            330.0,
            100.0,
            90.0,
            270.0
        );

        assertTrue(resolution.usedUnwindFallback());
        assertEquals(270.0, resolution.targetAngleDeg(), EPS);
        assertEquals(270.0, resolution.unwindTargetDeg(), EPS);
    }

    @Test
    // When both bounds have equal aim error, fallback should use the bound closest to the previous goal.
    void resolveTurretTargetDegrees_unwindFallbackBreaksAimErrorTieUsingCurrentAngle() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            0.0,
            250.0,
            90.0,
            270.0
        );

        assertTrue(resolution.usedUnwindFallback());
        assertEquals(270.0, resolution.targetAngleDeg(), EPS);
        assertEquals(270.0, resolution.unwindTargetDeg(), EPS);
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

    @Test
    // If clamping the current branch would leave the turret badly mis-aimed, choose the legal wrapped branch instead.
    void resolveTurretTargetDegrees_avoidsPinningAtLimitWhenWrappedBranchCanStillTrackTarget() {
        Shooter.TurretResolution resolution = Shooter.resolveTurretTargetDegrees(
            20.0,
            610.0,
            -450.0,
            630.0
        );

        assertFalse(resolution.usedUnwindFallback());
        assertEquals(380.0, resolution.targetAngleDeg(), EPS);
    }

    @Test
    void resolveTurretGoalState_convertsFieldRelativeAngleAndVelocityToRobotRelative() {
        Shooter.TurretGoalState goalState = Shooter.resolveTurretGoalState(
            Rotation2d.fromDegrees(90.0),
            0.75,
            Shooter.TurretReferenceFrame.FIELD_RELATIVE,
            Rotation2d.fromDegrees(30.0),
            0.25,
            40.0,
            -450.0,
            630.0
        );

        assertEquals(90.0, goalState.fieldRelativeAngleDeg(), EPS);
        assertEquals(0.75, goalState.fieldRelativeVelocityRotPerSec(), EPS);
        assertEquals(60.0, goalState.robotRelativeAngleDeg(), EPS);
        assertEquals(0.5, goalState.robotRelativeVelocityRotPerSec(), EPS);
        assertEquals(60.0, goalState.targetAngleDeg(), EPS);
        assertEquals(0.5, goalState.targetVelocityRotPerSec(), EPS);
        assertFalse(goalState.usedUnwindFallback());
    }

    @Test
    void resolveTurretGoalState_zeroesVelocityWhenUnwindFallbackIsUsed() {
        Shooter.TurretGoalState goalState = Shooter.resolveTurretGoalState(
            Rotation2d.fromDegrees(330.0),
            -0.6,
            Shooter.TurretReferenceFrame.FIELD_RELATIVE,
            Rotation2d.fromDegrees(0.0),
            0.0,
            100.0,
            90.0,
            270.0
        );

        assertTrue(goalState.usedUnwindFallback());
        assertEquals(270.0, goalState.targetAngleDeg(), EPS);
        assertEquals(0.0, goalState.targetVelocityRotPerSec(), EPS);
    }

    @Test
    void calculateTurretProfileSetpoint_preservesVelocitySign() {
        State setpoint = Shooter.calculateTurretProfileSetpoint(
            new State(0.40, 0.0),
            new State(0.20, -1.0),
            5.0,
            50.0,
            0.02
        );

        assertEquals(-1.0, setpoint.velocity, EPS);
        assertTrue(setpoint.position < 0.40);
        assertTrue(setpoint.position > 0.20);
    }
}
