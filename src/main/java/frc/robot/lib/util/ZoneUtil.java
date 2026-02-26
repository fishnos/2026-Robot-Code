package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.BLine.FlippingUtil;

public final class ZoneUtil {
    private static final double LOS_POINT_EPSILON_METERS = 1e-6;

    private ZoneUtil() {}

    public static boolean isPoseInZone(Pose2d pose, RectangleZone zone, boolean mirrorForAlliance) {
        return isPointInZone(pose.getTranslation(), zone, mirrorForAlliance);
    }

    public static boolean isPointInZone(Translation2d point, RectangleZone zone, boolean mirrorForAlliance) {
        Translation2d checkedPoint = maybeMirror(point, mirrorForAlliance);
        return zone.contains(checkedPoint);
    }

    public static boolean hasLineOfSight(
        Translation2d start,
        Translation2d end,
        Translation2d blockerPoint,
        boolean mirrorForAlliance
    ) {
        Translation2d checkedStart = maybeMirror(start, mirrorForAlliance);
        Translation2d checkedEnd = maybeMirror(end, mirrorForAlliance);
        Translation2d checkedBlocker = maybeMirror(blockerPoint, mirrorForAlliance);

        return !isPointOnLineSegment(checkedStart, checkedEnd, checkedBlocker);
    }

    private static Translation2d maybeMirror(Translation2d point, boolean mirrorForAlliance) {
        if (mirrorForAlliance && Constants.shouldFlipPath()) {
            return FlippingUtil.flipFieldPosition(point);
        }

        return point;
    }

    private static boolean isPointOnLineSegment(
        Translation2d start,
        Translation2d end,
        Translation2d point
    ) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double segmentLengthSquared = dx * dx + dy * dy;
        double epsilonSquared = LOS_POINT_EPSILON_METERS * LOS_POINT_EPSILON_METERS;

        if (segmentLengthSquared <= epsilonSquared) {
            return start.getDistance(point) <= LOS_POINT_EPSILON_METERS;
        }

        double apx = point.getX() - start.getX();
        double apy = point.getY() - start.getY();

        double cross = apx * dy - apy * dx;
        double segmentLength = Math.sqrt(segmentLengthSquared);
        if (Math.abs(cross) > LOS_POINT_EPSILON_METERS * segmentLength) {
            return false;
        }

        double dot = apx * dx + apy * dy;
        return dot >= -LOS_POINT_EPSILON_METERS && dot <= segmentLengthSquared + LOS_POINT_EPSILON_METERS;
    }
}
