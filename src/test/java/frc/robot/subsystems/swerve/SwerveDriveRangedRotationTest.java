package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SwerveDriveRangedRotationTest {
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
