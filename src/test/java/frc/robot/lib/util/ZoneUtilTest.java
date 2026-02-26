package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.BLine.FlippingUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class ZoneUtilTest {
    private static final RectangleZone TEST_ZONE = new RectangleZone(
        "test_zone",
        new Translation2d(1.0, 1.0),
        new Translation2d(3.0, 4.0)
    );

    @AfterEach
    void resetDriverStationState() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

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
    void isPointInZone_mirrorForAllianceUsesFlippedCoordinates() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();

        Translation2d bluePointInsideZone = new Translation2d(2.0, 2.0);
        Translation2d redEquivalentPoint = FlippingUtil.flipFieldPosition(bluePointInsideZone);

        assertTrue(ZoneUtil.isPointInZone(redEquivalentPoint, TEST_ZONE, true));
        assertFalse(ZoneUtil.isPointInZone(redEquivalentPoint, TEST_ZONE, false));
    }
}
