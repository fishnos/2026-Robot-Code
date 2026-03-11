package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ZoneConstants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import org.junit.jupiter.api.Test;

class ZoneUtilTest {
    private static final RectangleZone TEST_ZONE = new RectangleZone(
        "test_zone",
        new Translation2d(1.0, 1.0),
        new Translation2d(3.0, 4.0)
    );

    @Test
    void isPointInZone_returnsTrueForInteriorPoint() {
        assertTrue(ZoneUtil.isPointInZone(new Translation2d(2.0, 2.0), TEST_ZONE, false));
    }

    @Test
    void isPoseInZone_treatsBoundaryAsInside() {
        Pose2d boundaryPose = new Pose2d(3.0, 2.5, new Rotation2d());
        assertTrue(ZoneUtil.isPoseInZone(boundaryPose, TEST_ZONE, false));
    }

    @Test
    void isPointInZone_returnsFalseForPointOutsideBounds() {
        assertFalse(ZoneUtil.isPointInZone(new Translation2d(3.2, 2.0), TEST_ZONE, false));
    }

    @Test
    void isPointInAnyZone_allianceCompositeReturnsTrueInUpperSection() {
        assertTrue(
            ZoneUtil.isPointInAnyZone(
                new Translation2d(4.3, 7.5),
                ZoneConstants.Alliance.COMPOSITE,
                false
            )
        );
    }

    @Test
    void isPointInAnyZone_allianceCompositeReturnsTrueInMiddleSection() {
        assertTrue(
            ZoneUtil.isPointInAnyZone(
                new Translation2d(3.8, 4.0),
                ZoneConstants.Alliance.COMPOSITE,
                false
            )
        );
    }

    @Test
    void isPointInAnyZone_allianceCompositeReturnsFalseInCenterRightCutout() {
        assertFalse(
            ZoneUtil.isPointInAnyZone(
                new Translation2d(4.3, 4.0),
                ZoneConstants.Alliance.COMPOSITE,
                false
            )
        );
    }

    @Test
    void isPoseInAnyZone_allianceCompositeReturnsTrueInLowerSection() {
        assertTrue(
            ZoneUtil.isPoseInAnyZone(
                new Pose2d(4.2, 1.0, new Rotation2d()),
                ZoneConstants.Alliance.COMPOSITE,
                false
            )
        );
    }

    @Test
    void hasLineOfSight_returnsFalseWhenBlockerIsOnSegment() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSight(
            new Translation2d(0.0, 0.0),
            new Translation2d(4.0, 0.0),
            new Translation2d(2.0, 0.0),
            false
        );

        assertFalse(hasLineOfSight);
    }

    @Test
    void hasLineOfSight_returnsTrueWhenBlockerIsNotCollinear() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSight(
            new Translation2d(0.0, 0.0),
            new Translation2d(4.0, 0.0),
            new Translation2d(2.0, 0.5),
            false
        );

        assertTrue(hasLineOfSight);
    }

    @Test
    void hasLineOfSight_returnsTrueWhenBlockerIsCollinearButOutsideSegment() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSight(
            new Translation2d(0.0, 0.0),
            new Translation2d(4.0, 0.0),
            new Translation2d(5.0, 0.0),
            false
        );

        assertTrue(hasLineOfSight);
    }

    @Test
    void hasLineOfSight_handlesDegenerateSegment() {
        boolean blockedAtPoint = ZoneUtil.hasLineOfSight(
            new Translation2d(1.0, 1.0),
            new Translation2d(1.0, 1.0),
            new Translation2d(1.0 + 5e-7, 1.0),
            false
        );
        boolean clearAwayFromPoint = ZoneUtil.hasLineOfSight(
            new Translation2d(1.0, 1.0),
            new Translation2d(1.0, 1.0),
            new Translation2d(1.0 + 1e-3, 1.0),
            false
        );

        assertFalse(blockedAtPoint);
        assertTrue(clearAwayFromPoint);
    }

    @Test
    void hasLineOfSightWithMovingCircularBlocker_returnsFalseWhenMovingBallCrossesLos() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSightWithMovingCircularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(5.0, 0.0),
            new Translation2d(2.5, -1.0),
            new Translation2d(2.5, 1.0),
            0.2,
            false
        );

        assertFalse(hasLineOfSight);
    }

    @Test
    void hasLineOfSightWithMovingCircularBlocker_returnsTrueWhenPathStaysAwayFromLos() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSightWithMovingCircularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(5.0, 0.0),
            new Translation2d(2.5, 1.0),
            new Translation2d(2.5, 2.0),
            0.2,
            false
        );

        assertTrue(hasLineOfSight);
    }

    @Test
    void hasLineOfSightWithCircularBlocker_returnsFalseWhenBallRadiusIntersectsLos() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSightWithCircularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(5.0, 0.0),
            new Translation2d(2.5, 0.15),
            0.2,
            false
        );

        assertFalse(hasLineOfSight);
    }

    @Test
    void hasLineOfSightWithMovingCircularBlocker_zeroRadiusOnlyBlocksOnExactIntersection() {
        boolean clearWithZeroRadius = ZoneUtil.hasLineOfSightWithMovingCircularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(5.0, 0.0),
            new Translation2d(2.5, 0.001),
            new Translation2d(2.5, 1.0),
            0.0,
            false
        );
        boolean blockedWithZeroRadius = ZoneUtil.hasLineOfSightWithMovingCircularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(5.0, 0.0),
            new Translation2d(2.5, -1.0),
            new Translation2d(2.5, 1.0),
            0.0,
            false
        );

        assertTrue(clearWithZeroRadius);
        assertFalse(blockedWithZeroRadius);
    }

    @Test
    void hasLineOfSightWithRectangularBlocker_returnsFalseWhenSegmentCrossesZone() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSightWithRectangularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(10.0, 0.0),
            new RectangleZone(
                "hub_zone",
                new Translation2d(4.8, -0.3),
                new Translation2d(5.2, 0.3)
            ),
            0.0,
            false
        );

        assertFalse(hasLineOfSight);
    }

    @Test
    void hasLineOfSightWithRectangularBlocker_returnsTrueWhenSegmentMissesZone() {
        boolean hasLineOfSight = ZoneUtil.hasLineOfSightWithRectangularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(10.0, 0.0),
            new RectangleZone(
                "hub_zone",
                new Translation2d(4.8, 1.0),
                new Translation2d(5.2, 1.3)
            ),
            0.0,
            false
        );

        assertTrue(hasLineOfSight);
    }

    @Test
    void hasLineOfSightWithRectangularBlocker_paddingCanBlockNearMiss() {
        RectangleZone nearMissZone = new RectangleZone(
            "near_miss_hub_zone",
            new Translation2d(4.8, 0.25),
            new Translation2d(5.2, 0.35)
        );
        boolean clearWithoutPadding = ZoneUtil.hasLineOfSightWithRectangularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(10.0, 0.0),
            nearMissZone,
            0.0,
            false
        );
        boolean blockedWithPadding = ZoneUtil.hasLineOfSightWithRectangularBlocker(
            new Translation2d(0.0, 0.0),
            new Translation2d(10.0, 0.0),
            nearMissZone,
            0.3,
            false
        );

        assertTrue(clearWithoutPadding);
        assertFalse(blockedWithPadding);
    }

}
