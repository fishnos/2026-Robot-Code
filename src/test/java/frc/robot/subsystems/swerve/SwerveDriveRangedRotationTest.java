package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveRangedRotationTest {
    @Test
    void toggleTeleopDriveMode_flipsFieldRelativeToRobotRelative() {
        SwerveDrive.DesiredSystemState toggled = SwerveDrive.toggleTeleopDriveMode(
            SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE
        );

        assertEquals(SwerveDrive.DesiredSystemState.TELOP_ROBOT_RELATIVE, toggled);
    }

    @Test
    void toggleTeleopDriveMode_flipsRobotRelativeToFieldRelative() {
        SwerveDrive.DesiredSystemState toggled = SwerveDrive.toggleTeleopDriveMode(
            SwerveDrive.DesiredSystemState.TELOP_ROBOT_RELATIVE
        );

        assertEquals(SwerveDrive.DesiredSystemState.TELOP_FIELD_RELATIVE, toggled);
    }

    @Test
    void toggleTeleopDriveMode_leavesNonTeleopStatesUnchanged() {
        SwerveDrive.DesiredSystemState toggled = SwerveDrive.toggleTeleopDriveMode(
            SwerveDrive.DesiredSystemState.FOLLOW_PATH
        );

        assertEquals(SwerveDrive.DesiredSystemState.FOLLOW_PATH, toggled);
    }

    @Test
    void applyRobotRelativeTeleopHeadingOffset_rotatesTranslationCounterClockwise() {
        ChassisSpeeds rotatedSpeeds = SwerveDrive.applyRobotRelativeTeleopHeadingOffset(
            new ChassisSpeeds(1.0, 0.0, 2.0),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(25.0)
        );

        assertEquals(Math.cos(Math.toRadians(25.0)), rotatedSpeeds.vxMetersPerSecond, 1e-9);
        assertEquals(Math.sin(Math.toRadians(25.0)), rotatedSpeeds.vyMetersPerSecond, 1e-9);
        assertEquals(2.0, rotatedSpeeds.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void applyRobotRelativeTeleopHeadingOffset_preservesSpeedsWhenOffsetIsZero() {
        ChassisSpeeds rotatedSpeeds = SwerveDrive.applyRobotRelativeTeleopHeadingOffset(
            new ChassisSpeeds(0.5, -0.25, 1.0),
            new edu.wpi.first.math.geometry.Rotation2d()
        );

        assertEquals(0.5, rotatedSpeeds.vxMetersPerSecond, 1e-9);
        assertEquals(-0.25, rotatedSpeeds.vyMetersPerSecond, 1e-9);
        assertEquals(1.0, rotatedSpeeds.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void resolveRangedRotationState_staysReturningUntilInsideNominalBuffer() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING,
            true,
            false
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_transitionsToNominalWhenBufferedRangeRecovered() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL, state);
    }

    @Test
    void resolveRangedRotationState_entersReturningWhenOutsideRange() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL,
            false,
            false
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_usesNominalWhenInRangeAndNotReturning() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.NONE,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL, state);
    }

    @Test
    void resolveRangedRotationState_cappedModeUsesCappedNominalState() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.NONE,
            true,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_NOMINAL, state);
    }

    @Test
    void resolveRangedRotationState_cappedModeUsesCappedReturningState() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.NONE,
            false,
            false,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_cappedReturningStaysReturningUntilBufferedRecovery() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING,
            true,
            false,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING, state);
    }

    @Test
    void shouldApplyRangedRotationOmegaVelocityCap_keepsCapInNominalState() {
        assertTrue(SwerveDrive.shouldApplyRangedRotationOmegaVelocityCap(false, true));
    }

    @Test
    void shouldApplyRangedRotationOmegaVelocityCap_bypassesCapWhileReturning() {
        assertFalse(SwerveDrive.shouldApplyRangedRotationOmegaVelocityCap(true, true));
    }

    @Test
    void resolveBufferedRotationRange_clampsBufferToHalfWidthForNarrowRanges() {
        SwerveDrive.RotationRangeBounds bounds = SwerveDrive.resolveBufferedRotationRange(
            Math.toRadians(10.0),
            Math.toRadians(20.0),
            Math.toRadians(15.0)
        );

        assertEquals(Math.toRadians(15.0), bounds.minRad(), 1e-9);
        assertEquals(Math.toRadians(15.0), bounds.maxRad(), 1e-9);
        assertEquals(Math.toRadians(5.0), bounds.bufferRad(), 1e-9);
    }

    @Test
    void resolveBufferedRotationRange_keepsRequestedBufferWhenRangeIsWideEnough() {
        SwerveDrive.RotationRangeBounds bounds = SwerveDrive.resolveBufferedRotationRange(
            Math.toRadians(-30.0),
            Math.toRadians(30.0),
            Math.toRadians(10.0)
        );

        assertEquals(Math.toRadians(-20.0), bounds.minRad(), 1e-9);
        assertEquals(Math.toRadians(20.0), bounds.maxRad(), 1e-9);
        assertEquals(Math.toRadians(10.0), bounds.bufferRad(), 1e-9);
    }

    @Test
    void applyClosingRecoveryBoundaryFeedforward_enforcesMinimumOmegaForClosingMinBoundary() {
        SwerveDrive.RecoveryBoundaryFeedforward feedforward =
            SwerveDrive.applyClosingRecoveryBoundaryFeedforward(
                -0.5,
                Math.toRadians(-25.0),
                Math.toRadians(-30.0),
                Math.toRadians(30.0),
                Math.toRadians(-15.0),
                Math.toRadians(15.0),
                1.25,
                0.0,
                4.0
            );

        assertEquals(1.25, feedforward.adjustedOmega(), 1e-9);
        assertEquals(1.25, feedforward.feedforwardOmegaRadPerSec(), 1e-9);
        assertEquals(SwerveDrive.RecoveryBoundaryFeedforward.MIN_BOUNDARY, feedforward.activeBoundary());
    }

    @Test
    void applyClosingRecoveryBoundaryFeedforward_enforcesMinimumOmegaForClosingMaxBoundary() {
        SwerveDrive.RecoveryBoundaryFeedforward feedforward =
            SwerveDrive.applyClosingRecoveryBoundaryFeedforward(
                0.25,
                Math.toRadians(25.0),
                Math.toRadians(-30.0),
                Math.toRadians(30.0),
                Math.toRadians(-15.0),
                Math.toRadians(15.0),
                0.0,
                -1.75,
                4.0
            );

        assertEquals(-1.75, feedforward.adjustedOmega(), 1e-9);
        assertEquals(-1.75, feedforward.feedforwardOmegaRadPerSec(), 1e-9);
        assertEquals(SwerveDrive.RecoveryBoundaryFeedforward.MAX_BOUNDARY, feedforward.activeBoundary());
    }

    @Test
    void applyClosingRecoveryBoundaryFeedforward_doesNotOverrideFasterDriverEscapeCommand() {
        SwerveDrive.RecoveryBoundaryFeedforward feedforward =
            SwerveDrive.applyClosingRecoveryBoundaryFeedforward(
                -2.5,
                Math.toRadians(25.0),
                Math.toRadians(-30.0),
                Math.toRadians(30.0),
                Math.toRadians(-15.0),
                Math.toRadians(15.0),
                0.0,
                -1.0,
                4.0
            );

        assertEquals(-2.5, feedforward.adjustedOmega(), 1e-9);
        assertEquals(-1.0, feedforward.feedforwardOmegaRadPerSec(), 1e-9);
    }

    @Test
    void applyClosingRecoveryBoundaryFeedforward_ignoresOpeningBoundaryMotion() {
        SwerveDrive.RecoveryBoundaryFeedforward feedforward =
            SwerveDrive.applyClosingRecoveryBoundaryFeedforward(
                -0.5,
                Math.toRadians(-25.0),
                Math.toRadians(-30.0),
                Math.toRadians(30.0),
                Math.toRadians(-15.0),
                Math.toRadians(15.0),
                -1.25,
                0.0,
                4.0
            );

        assertEquals(-0.5, feedforward.adjustedOmega(), 1e-9);
        assertEquals(0.0, feedforward.feedforwardOmegaRadPerSec(), 1e-9);
        assertEquals(SwerveDrive.RecoveryBoundaryFeedforward.MIN_BOUNDARY, feedforward.activeBoundary());
    }

    @Test
    void applyClosingRecoveryBoundaryFeedforward_isInactiveOutsideRecoveryBuffer() {
        SwerveDrive.RecoveryBoundaryFeedforward feedforward =
            SwerveDrive.applyClosingRecoveryBoundaryFeedforward(
                0.75,
                0.0,
                Math.toRadians(-30.0),
                Math.toRadians(30.0),
                Math.toRadians(-15.0),
                Math.toRadians(15.0),
                1.25,
                -1.75,
                4.0
            );

        assertEquals(0.75, feedforward.adjustedOmega(), 1e-9);
        assertEquals(0.0, feedforward.feedforwardOmegaRadPerSec(), 1e-9);
        assertEquals(SwerveDrive.RecoveryBoundaryFeedforward.NO_ACTIVE_BOUNDARY, feedforward.activeBoundary());
    }

    @Test
    void resolveReturnToRangeTargetAngle_targetsNearestRecoveryBound() {
        double targetNearMin = SwerveDrive.resolveReturnToRangeTargetAngle(
            Math.toRadians(-25.0),
            Math.toRadians(-15.0),
            Math.toRadians(15.0)
        );
        double targetNearMax = SwerveDrive.resolveReturnToRangeTargetAngle(
            Math.toRadians(22.0),
            Math.toRadians(-15.0),
            Math.toRadians(15.0)
        );

        assertEquals(Math.toRadians(-15.0), targetNearMin, 1e-9);
        assertEquals(Math.toRadians(15.0), targetNearMax, 1e-9);
    }

    @Test
    void resolveReturnToRangeTargetAngle_collapsesToSinglePointForNarrowRanges() {
        double target = SwerveDrive.resolveReturnToRangeTargetAngle(
            Math.toRadians(50.0),
            Math.toRadians(15.0),
            Math.toRadians(15.0)
        );

        assertEquals(Math.toRadians(15.0), target, 1e-9);
    }

    @Test
    void limitOmegaForRange_usesOuterSafetyBufferRatherThanInnerRecoveryBuffer() {
        double limitedOmega = SwerveDrive.limitOmegaForRange(
            10.0,
            Math.toRadians(80.0),
            0.0,
            Math.toRadians(5.0),
            Math.toRadians(85.0),
            4.0
        );

        assertEquals(Math.sqrt(2 * 4.0 * Math.toRadians(5.0)), limitedOmega, 1e-9);
    }

    @Test
    void limitOmegaForRange_reversesDirectionWhenAlreadyBeyondSafeRange() {
        double limitedOmega = SwerveDrive.limitOmegaForRange(
            3.0,
            Math.toRadians(95.0),
            0.0,
            Math.toRadians(-80.0),
            Math.toRadians(80.0),
            4.0
        );

        assertEquals(0.0, limitedOmega, 1e-9);
    }

    @Test
    void limitOmegaForRange_accountsForStoppingDistanceFromCurrentOmega() {
        double limitedOmega = SwerveDrive.limitOmegaForRange(
            6.0,
            Math.toRadians(70.0),
            2.0,
            Math.toRadians(-80.0),
            Math.toRadians(80.0),
            4.0
        );
        double stoppingDistance = (2.0 * 2.0) / (2 * 4.0);
        double expected = Math.sqrt(2 * 4.0 * Math.max(0.0, Math.toRadians(10.0) - stoppingDistance));

        assertEquals(expected, limitedOmega, 1e-9);
    }

    @Test
    void resolveRangedRotationState_cappedReturningTransitionsToNominalWhenBufferedRecovered() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING,
            true,
            true,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_NOMINAL, state);
    }

    @Test
    void resolveRangedRotationState_cappedNominalTransitionsToReturningWhenOutOfRange() {
        SwerveDrive.CurrentOmegaOverrideState state = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_NOMINAL,
            false,
            false,
            true
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING, state);
    }

    @Test
    void resolveRangedRotationState_switchingFromCappedReturningToUncappedKeepsReturningSemantics() {
        SwerveDrive.CurrentOmegaOverrideState stateOutsideNominal = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING,
            true,
            false,
            false
        );
        SwerveDrive.CurrentOmegaOverrideState stateInsideNominal = SwerveDrive.resolveRangedRotationState(
            SwerveDrive.CurrentOmegaOverrideState.RANGED_CAPPED_RETURNING,
            true,
            true,
            false
        );

        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_RETURNING, stateOutsideNominal);
        assertEquals(SwerveDrive.CurrentOmegaOverrideState.RANGED_NOMINAL, stateInsideNominal);
    }

    @Test
    void resolveAbsoluteRotationRangeDegrees_keepsRangeNearReference() {
        SwerveDrive.AbsoluteRotationRange range = SwerveDrive.resolveAbsoluteRotationRangeDegrees(
            -20.0,
            20.0,
            5.0
        );

        assertEquals(-20.0, range.minAbsDeg(), 1e-9);
        assertEquals(20.0, range.maxAbsDeg(), 1e-9);
        assertEquals(0.0, range.midpointDeg(), 1e-9);
        assertEquals(0L, range.turnShift());
    }

    @Test
    void resolveAbsoluteRotationRangeDegrees_unwrapsToNearestTurnForHighReference() {
        SwerveDrive.AbsoluteRotationRange range = SwerveDrive.resolveAbsoluteRotationRangeDegrees(
            -20.0,
            20.0,
            350.0
        );

        assertEquals(340.0, range.minAbsDeg(), 1e-9);
        assertEquals(380.0, range.maxAbsDeg(), 1e-9);
        assertEquals(360.0, range.midpointDeg(), 1e-9);
        assertEquals(1L, range.turnShift());
    }

    @Test
    void resolveAbsoluteRotationRangeDegrees_swapsReversedBoundsBeforeProjection() {
        SwerveDrive.AbsoluteRotationRange range = SwerveDrive.resolveAbsoluteRotationRangeDegrees(
            20.0,
            -20.0,
            350.0
        );

        assertEquals(340.0, range.minAbsDeg(), 1e-9);
        assertEquals(380.0, range.maxAbsDeg(), 1e-9);
        assertEquals(360.0, range.midpointDeg(), 1e-9);
        assertEquals(1L, range.turnShift());
    }

    @Test
    void resolveAbsoluteRotationRangeDegrees_handlesNegativeReferenceTurnSelection() {
        SwerveDrive.AbsoluteRotationRange range = SwerveDrive.resolveAbsoluteRotationRangeDegrees(
            -30.0,
            30.0,
            -725.0
        );

        assertEquals(-750.0, range.minAbsDeg(), 1e-9);
        assertEquals(-690.0, range.maxAbsDeg(), 1e-9);
        assertEquals(-720.0, range.midpointDeg(), 1e-9);
        assertEquals(-2L, range.turnShift());
    }

    @Test
    void resolveWrappedRotationRangeDegrees_normalizesAndSwapsWhenNeeded() {
        SwerveDrive.WrappedRotationRangeResolution resolution = SwerveDrive.resolveWrappedRotationRangeDegrees(
            190.0,
            540.0,
            170.0
        );

        assertEquals(-180.0, resolution.normalizedMinDeg(), 1e-9);
        assertEquals(-170.0, resolution.normalizedMaxDeg(), 1e-9);
        assertTrue(resolution.swappedInputs());
        assertEquals(180.0, resolution.absoluteRange().minAbsDeg(), 1e-9);
        assertEquals(190.0, resolution.absoluteRange().maxAbsDeg(), 1e-9);
    }

    @Test
    void resolveWrappedRotationRangeDegrees_projectsToNearestReferenceTurn() {
        SwerveDrive.WrappedRotationRangeResolution resolution = SwerveDrive.resolveWrappedRotationRangeDegrees(
            -20.0,
            20.0,
            722.0
        );

        assertFalse(resolution.swappedInputs());
        assertEquals(700.0, resolution.absoluteRange().minAbsDeg(), 1e-9);
        assertEquals(740.0, resolution.absoluteRange().maxAbsDeg(), 1e-9);
        assertEquals(2L, resolution.absoluteRange().turnShift());
    }

    @Test
    void resolveWrappedRotationRangeDegrees_supportsZeroWidthRange() {
        SwerveDrive.WrappedRotationRangeResolution resolution = SwerveDrive.resolveWrappedRotationRangeDegrees(
            45.0,
            45.0,
            810.0
        );

        assertFalse(resolution.swappedInputs());
        assertEquals(45.0, resolution.normalizedMinDeg(), 1e-9);
        assertEquals(45.0, resolution.normalizedMaxDeg(), 1e-9);
        assertEquals(765.0, resolution.absoluteRange().minAbsDeg(), 1e-9);
        assertEquals(765.0, resolution.absoluteRange().maxAbsDeg(), 1e-9);
    }

    @Test
    void resolveWrappedRotationRangeDegrees_treatsPlusAndMinus180AsEquivalentWithoutSwap() {
        SwerveDrive.WrappedRotationRangeResolution resolution = SwerveDrive.resolveWrappedRotationRangeDegrees(
            180.0,
            -180.0,
            0.0
        );

        assertFalse(resolution.swappedInputs());
        assertEquals(-180.0, resolution.normalizedMinDeg(), 1e-9);
        assertEquals(-180.0, resolution.normalizedMaxDeg(), 1e-9);
        assertEquals(180.0, resolution.absoluteRange().minAbsDeg(), 1e-9);
        assertEquals(180.0, resolution.absoluteRange().maxAbsDeg(), 1e-9);
    }

    @Test
    void resolveAccumulatedRotationRangeDegrees_preservesMultiWrapRanges() {
        SwerveDrive.AccumulatedRotationRangeResolution resolution =
            SwerveDrive.resolveAccumulatedRotationRangeDegrees(-400.0, 500.0);

        assertEquals(-400.0, resolution.minAbsDeg(), 1e-9);
        assertEquals(500.0, resolution.maxAbsDeg(), 1e-9);
        assertFalse(resolution.swappedInputs());
    }

    @Test
    void resolveAccumulatedRotationRangeDegrees_swapsReversedBounds() {
        SwerveDrive.AccumulatedRotationRangeResolution resolution =
            SwerveDrive.resolveAccumulatedRotationRangeDegrees(50.0, -10.0);

        assertEquals(-10.0, resolution.minAbsDeg(), 1e-9);
        assertEquals(50.0, resolution.maxAbsDeg(), 1e-9);
        assertTrue(resolution.swappedInputs());
    }

    @Test
    void resolveAccumulatedRotationRangeDegrees_keepsEqualBoundsWithoutSwap() {
        SwerveDrive.AccumulatedRotationRangeResolution resolution =
            SwerveDrive.resolveAccumulatedRotationRangeDegrees(42.0, 42.0);

        assertEquals(42.0, resolution.minAbsDeg(), 1e-9);
        assertEquals(42.0, resolution.maxAbsDeg(), 1e-9);
        assertFalse(resolution.swappedInputs());
    }

    @Test
    void normalizeWrappedDegrees_wrapsToMinus180To180Interval() {
        assertEquals(-170.0, SwerveDrive.normalizeWrappedDegrees(190.0), 1e-9);
        assertEquals(-180.0, SwerveDrive.normalizeWrappedDegrees(540.0), 1e-9);
        assertEquals(170.0, SwerveDrive.normalizeWrappedDegrees(-190.0), 1e-9);
    }

    @Test
    void normalizeWrappedDegrees_mapsPositiveAndNegative180ToNegative180() {
        assertEquals(-180.0, SwerveDrive.normalizeWrappedDegrees(180.0), 1e-9);
        assertEquals(-180.0, SwerveDrive.normalizeWrappedDegrees(-180.0), 1e-9);
    }

    @Test
    void normalizeWrappedDegrees_isPeriodicEveryFullTurn() {
        double expected = SwerveDrive.normalizeWrappedDegrees(33.0);

        for (int turns = -4; turns <= 4; turns++) {
            double candidate = 33.0 + turns * 360.0;
            assertEquals(expected, SwerveDrive.normalizeWrappedDegrees(candidate), 1e-9);
        }
    }
}
