package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.util.ShotCalculator.ShotData;
import org.junit.jupiter.api.Test;

class SuperstructureShotReadinessTest {
    @Test
    // Ready when both impact error and distance bounds are satisfied.
    void isShotReady_trueWhenImpactAndDistanceAreValid() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 4.0, 2.0, 6.0);
        assertTrue(ready);
    }

    @Test
    // Exceeding impact tolerance should fail readiness.
    void isShotReady_falseWhenImpactExceedsTolerance() {
        boolean ready = Superstructure.isShotReady(0.31, 0.3, 4.0, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances below configured minimum should fail readiness.
    void isShotReady_falseWhenDistanceBelowMinimum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 1.99, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Distances above configured maximum should fail readiness.
    void isShotReady_falseWhenDistanceAboveMaximum() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 6.01, 2.0, 6.0);
        assertFalse(ready);
    }

    @Test
    // Min/max boundaries are inclusive.
    void isShotReady_allowsBoundaryDistances() {
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 2.0, 2.0, 6.0));
        assertTrue(Superstructure.isShotReady(0.05, 0.3, 6.0, 2.0, 6.0));
    }

    @Test
    void isShotReady_falseWhenDistanceBoundsAreReversed() {
        boolean ready = Superstructure.isShotReady(0.05, 0.3, 4.0, 6.0, 2.0);
        assertFalse(ready);
    }

    @Test
    void isShotReady_falseWhenImpactErrorIsNotFinite() {
        assertFalse(Superstructure.isShotReady(Double.NaN, 0.3, 4.0, 2.0, 6.0));
        assertFalse(Superstructure.isShotReady(Double.POSITIVE_INFINITY, 0.3, 4.0, 2.0, 6.0));
    }

    @Test
    void isDistanceInRange_matchesBoundaryAndOutsideBehavior() {
        assertTrue(Superstructure.isDistanceInRange(2.0, 2.0, 6.0));
        assertTrue(Superstructure.isDistanceInRange(4.0, 2.0, 6.0));
        assertTrue(Superstructure.isDistanceInRange(6.0, 2.0, 6.0));
        assertFalse(Superstructure.isDistanceInRange(1.99, 2.0, 6.0));
        assertFalse(Superstructure.isDistanceInRange(6.01, 2.0, 6.0));
    }

    @Test
    void isDistanceInRange_falseForReversedBoundsAndNaNDistance() {
        assertFalse(Superstructure.isDistanceInRange(4.0, 6.0, 2.0));
        assertFalse(Superstructure.isDistanceInRange(Double.NaN, 2.0, 6.0));
    }

    @Test
    void didShotReachTargetHeight_trueWithinReachEpsilon() {
        assertTrue(Superstructure.didShotReachTargetHeight(new Translation3d(1.0, 2.0, 1.8009), 1.8010));
    }

    @Test
    void didShotReachTargetHeight_falseWhenBelowTargetBeyondEpsilon() {
        assertFalse(Superstructure.didShotReachTargetHeight(new Translation3d(1.0, 2.0, 0.10), 1.80));
    }

    @Test
    void calculateTurretFieldRelativeErrorDegrees_isZeroWhenRobotAndTurretComposeToTarget() {
        double errorDeg = Superstructure.calculateTurretFieldRelativeErrorDegrees(
            30.0 / 360.0,
            Rotation2d.fromDegrees(45.0),
            Rotation2d.fromDegrees(75.0)
        );

        assertEquals(0.0, errorDeg, 1e-9);
    }

    @Test
    void calculateTurretFieldRelativeErrorDegrees_wrapsAcrossSignedAngleBoundary() {
        double errorDeg = Superstructure.calculateTurretFieldRelativeErrorDegrees(
            20.0 / 360.0,
            Rotation2d.fromDegrees(170.0),
            Rotation2d.fromDegrees(-170.0)
        );

        assertEquals(0.0, errorDeg, 1e-9);
    }

    @Test
    void calculateTurretFieldRelativeErrorDegrees_reportsSmallFieldFrameAimDelta() {
        double errorDeg = Superstructure.calculateTurretFieldRelativeErrorDegrees(
            30.0 / 360.0,
            Rotation2d.fromDegrees(45.0),
            Rotation2d.fromDegrees(80.0)
        );

        assertEquals(5.0, errorDeg, 1e-9);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_usesLastInRangeWhenTooClose() {
        ShotData mostRecentTooClose = shotData(1.5);
        ShotData lastInRange = shotData(2.5);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentTooClose,
            lastInRange,
            2.0
        );

        assertSame(lastInRange, selected);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_fallsBackToMostRecentWhenNoLastInRange() {
        ShotData mostRecentTooClose = shotData(1.5);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentTooClose,
            null,
            2.0
        );

        assertSame(mostRecentTooClose, selected);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_usesMostRecentWhenNotTooClose() {
        ShotData mostRecentOutOfRangeFar = shotData(7.0);
        ShotData lastInRange = shotData(4.0);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentOutOfRangeFar,
            lastInRange,
            2.0
        );

        assertSame(mostRecentOutOfRangeFar, selected);
    }

    @Test
    void selectShotDataWithMinDistanceGuard_keepsMostRecentAtExactMinimumDistance() {
        ShotData mostRecentAtMinimum = shotData(2.0);
        ShotData lastInRange = shotData(2.5);

        ShotData selected = Superstructure.selectShotDataWithMinDistanceGuard(
            mostRecentAtMinimum,
            lastInRange,
            2.0
        );

        assertSame(mostRecentAtMinimum, selected);
    }

    @Test
    void isLastInRangeShotDataValid_falseWhenTimestampIsNotFinite() {
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, Double.NaN, 1.0));
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, Double.NEGATIVE_INFINITY, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_trueWhenShotAgeAtOrBelowMaxAge() {
        assertTrue(Superstructure.isLastInRangeShotDataValid(10.0, 9.0, 1.0));
        assertTrue(Superstructure.isLastInRangeShotDataValid(10.0, 9.1, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_falseWhenShotAgeExceedsMaxAge() {
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, 8.9, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_currentBehaviorTreatsFutureTimestampAsValid() {
        assertTrue(Superstructure.isLastInRangeShotDataValid(10.0, 10.2, 1.0));
    }

    @Test
    void isLastInRangeShotDataValid_falseWhenMaxAgeIsNegative() {
        assertFalse(Superstructure.isLastInRangeShotDataValid(10.0, 10.0, -0.1));
    }

    @Test
    void calculateTurretRotationMargins_usesMeasuredTravelMinusBuffer() {
        Superstructure.TurretRotationMargins margins = Superstructure.calculateTurretRotationMargins(
            10.0,
            -40.0,
            80.0,
            20.0
        );

        assertEquals(30.0, margins.marginTowardMinDeg(), 1e-9);
        assertEquals(50.0, margins.marginTowardMaxDeg(), 1e-9);
        assertTrue(margins.valid());
    }

    @Test
    void calculateTurretRotationMargins_clampsExhaustedSideToZeroAndRemainsValidWhenOppositeSideHasTravel() {
        Superstructure.TurretRotationMargins margins = Superstructure.calculateTurretRotationMargins(
            -30.0,
            -40.0,
            80.0,
            20.0
        );

        assertEquals(0.0, margins.marginTowardMinDeg(), 1e-9);
        assertEquals(90.0, margins.marginTowardMaxDeg(), 1e-9);
        assertTrue(margins.valid());
    }

    @Test
    void calculateTurretRotationMargins_zeroRemainingMarginOnOneSideIsValid() {
        Superstructure.TurretRotationMargins margins = Superstructure.calculateTurretRotationMargins(
            -20.0,
            -40.0,
            80.0,
            20.0
        );

        assertEquals(0.0, margins.marginTowardMinDeg(), 1e-9);
        assertEquals(80.0, margins.marginTowardMaxDeg(), 1e-9);
        assertTrue(margins.valid());
    }

    @Test
    void calculateTurretRotationMargins_marksInvalidWhenBufferedTravelIsExhaustedOnBothSides() {
        Superstructure.TurretRotationMargins margins = Superstructure.calculateTurretRotationMargins(
            -35.0,
            -40.0,
            -30.0,
            10.0
        );

        assertEquals(0.0, margins.marginTowardMinDeg(), 1e-9);
        assertEquals(0.0, margins.marginTowardMaxDeg(), 1e-9);
        assertFalse(margins.valid());
    }

    @Test
    void calculateTurretRotationMargins_negativeBufferExpandsMargins() {
        Superstructure.TurretRotationMargins margins = Superstructure.calculateTurretRotationMargins(
            0.0,
            -30.0,
            50.0,
            -5.0
        );

        assertEquals(35.0, margins.marginTowardMinDeg(), 1e-9);
        assertEquals(55.0, margins.marginTowardMaxDeg(), 1e-9);
        assertTrue(margins.valid());
    }

    @Test
    void calculateAccumulatedYawRotationRange_appliesMinMaxMarginsAroundYawWithCorrectSigns() {
        Superstructure.AccumulatedYawRotationRange range = Superstructure.calculateAccumulatedYawRotationRange(
            300.0,
            40.0,
            55.0
        );

        assertEquals(245.0, range.minAbsDeg(), 1e-9);
        assertEquals(340.0, range.maxAbsDeg(), 1e-9);
    }

    @Test
    void calculateAccumulatedYawRotationRange_supportsLargeAccumulatedYawValues() {
        Superstructure.AccumulatedYawRotationRange range = Superstructure.calculateAccumulatedYawRotationRange(
            1080.0,
            120.0,
            90.0
        );

        assertEquals(990.0, range.minAbsDeg(), 1e-9);
        assertEquals(1200.0, range.maxAbsDeg(), 1e-9);
    }

    @Test
    void calculateAccumulatedYawRotationRange_asymmetricTurretHeadroomMatchesExpectedYawBounds() {
        // Example: turret range [-90, 90], current turret +80 -> toward min=170, toward max=10.
        // Robot yaw should be allowed in [yaw-10, yaw+170].
        Superstructure.AccumulatedYawRotationRange range = Superstructure.calculateAccumulatedYawRotationRange(
            0.0,
            170.0,
            10.0
        );

        assertEquals(-10.0, range.minAbsDeg(), 1e-9);
        assertEquals(170.0, range.maxAbsDeg(), 1e-9);
    }

    @Test
    void resolveTargetForZoneConstraints_preservesUserSelectedTarget() {
        assertEquals(
            Superstructure.TargetState.HUB,
            Superstructure.resolveTargetForZoneConstraints(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.CURRENT_ALLIANCE
            )
        );
        assertEquals(
            Superstructure.TargetState.HUB,
            Superstructure.resolveTargetForZoneConstraints(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.NEUTRAL
            )
        );
        assertEquals(
            Superstructure.TargetState.HUB,
            Superstructure.resolveTargetForZoneConstraints(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.OPPOSING_ALLIANCE
            )
        );
        assertEquals(
            Superstructure.TargetState.HUB,
            Superstructure.resolveTargetForZoneConstraints(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.UNKNOWN
            )
        );
    }

    @Test
    void isTargetAllowedInZone_hubOnlyAllowedInCurrentAllianceZone() {
        assertTrue(
            Superstructure.isTargetAllowedInZone(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.CURRENT_ALLIANCE
            )
        );
        assertFalse(
            Superstructure.isTargetAllowedInZone(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.NEUTRAL
            )
        );
        assertFalse(
            Superstructure.isTargetAllowedInZone(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.OPPOSING_ALLIANCE
            )
        );
        assertFalse(
            Superstructure.isTargetAllowedInZone(
                Superstructure.TargetState.HUB,
                Superstructure.RobotFieldZone.UNKNOWN
            )
        );
    }

    @Test
    void resolveTargetForZoneConstraints_allPassTargetsAreAllowedFromAnyZone() {
        Superstructure.TargetState[] passTargets = {
            Superstructure.TargetState.PASS_ALLIANCE_TOP,
            Superstructure.TargetState.PASS_ALLIANCE_CENTER,
            Superstructure.TargetState.PASS_ALLIANCE_BOTTOM,
            Superstructure.TargetState.PASS_NEUTRAL_TOP,
            Superstructure.TargetState.PASS_NEUTRAL_CENTER,
            Superstructure.TargetState.PASS_NEUTRAL_BOTTOM
        };
        Superstructure.RobotFieldZone[] zones = {
            Superstructure.RobotFieldZone.CURRENT_ALLIANCE,
            Superstructure.RobotFieldZone.NEUTRAL,
            Superstructure.RobotFieldZone.OPPOSING_ALLIANCE,
            Superstructure.RobotFieldZone.UNKNOWN
        };

        for (Superstructure.TargetState passTarget : passTargets) {
            for (Superstructure.RobotFieldZone zone : zones) {
                assertEquals(
                    passTarget,
                    Superstructure.resolveTargetForZoneConstraints(passTarget, zone)
                );
            }
        }
    }

    @Test
    void isPassLineOfSightClear_checksHubAndTowerBlockersWithMirroring() {
        Translation2d shooter = new Translation2d(0.0, 0.0);
        Translation2d target = new Translation2d(10.0, 0.0);
        RectangleZone clearHub = new RectangleZone(
            "clear_hub",
            new Translation2d(5.0, 1.0),
            new Translation2d(6.0, 2.0)
        );
        RectangleZone clearFlippedHub = new RectangleZone(
            "clear_flipped_hub",
            new Translation2d(5.0, -2.0),
            new Translation2d(6.0, -1.0)
        );
        RectangleZone blockingHub = new RectangleZone(
            "blocking_hub",
            new Translation2d(4.8, -0.2),
            new Translation2d(5.2, 0.2)
        );
        RectangleZone clearTower = new RectangleZone(
            "clear_tower",
            new Translation2d(2.0, 1.0),
            new Translation2d(3.0, 2.0)
        );
        RectangleZone clearFlippedTower = new RectangleZone(
            "clear_flipped_tower",
            new Translation2d(2.0, -2.0),
            new Translation2d(3.0, -1.0)
        );
        RectangleZone blockingTower = new RectangleZone(
            "blocking_tower",
            new Translation2d(2.4, -0.2),
            new Translation2d(2.8, 0.2)
        );

        assertTrue(
            Superstructure.isPassLineOfSightClear(
                shooter,
                target,
                clearHub,
                clearFlippedHub,
                clearTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isPassLineOfSightClear(
                shooter,
                target,
                blockingHub,
                clearFlippedHub,
                clearTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isPassLineOfSightClear(
                shooter,
                target,
                clearHub,
                blockingHub,
                clearTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isPassLineOfSightClear(
                shooter,
                target,
                clearHub,
                clearFlippedHub,
                blockingTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isPassLineOfSightClear(
                shooter,
                target,
                clearHub,
                clearFlippedHub,
                clearTower,
                blockingTower,
                0.12
            )
        );
    }

    @Test
    void isHubLineOfSightClear_checksMirroredTowerBlockers() {
        Translation2d shooter = new Translation2d(0.0, 0.0);
        Translation2d target = new Translation2d(10.0, 0.0);
        RectangleZone clearTower = new RectangleZone(
            "clear_tower",
            new Translation2d(2.0, 1.0),
            new Translation2d(3.0, 2.0)
        );
        RectangleZone clearFlippedTower = new RectangleZone(
            "clear_flipped_tower",
            new Translation2d(2.0, -2.0),
            new Translation2d(3.0, -1.0)
        );
        RectangleZone blockingTower = new RectangleZone(
            "blocking_tower",
            new Translation2d(2.4, -0.2),
            new Translation2d(2.8, 0.2)
        );

        assertTrue(
            Superstructure.isHubLineOfSightClear(
                shooter,
                target,
                clearTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isHubLineOfSightClear(
                shooter,
                target,
                blockingTower,
                clearFlippedTower,
                0.12
            )
        );
        assertFalse(
            Superstructure.isHubLineOfSightClear(
                shooter,
                target,
                clearTower,
                blockingTower,
                0.12
            )
        );
    }

    private static ShotData shotData(double effectiveDistanceMeters) {
        return new ShotData(
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(0.0),
            0.0,
            0.0,
            0.0,
            effectiveDistanceMeters,
            new Translation2d()
        );
    }
}
