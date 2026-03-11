package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.ZoneConstants.RectangleZone;
import frc.robot.lib.BLine.FlippingUtil;
import java.util.Collection;

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

    public static boolean isPoseInAnyZone(Pose2d pose, Collection<RectangleZone> zones, boolean mirrorForAlliance) {
        return isPointInAnyZone(pose.getTranslation(), zones, mirrorForAlliance);
    }

    public static boolean isPoseInOpposingAllianceZone(Pose2d pose, Collection<RectangleZone> zones) {
        return isPointInAnyZoneWithExplicitFlip(pose.getTranslation(), zones, shouldFlipForOpposingAllianceZone());
    }

    public static boolean isPointInAnyZone(
        Translation2d point,
        Collection<RectangleZone> zones,
        boolean mirrorForAlliance
    ) {
        Translation2d checkedPoint = maybeMirror(point, mirrorForAlliance);
        for (RectangleZone zone : zones) {
            if (zone.contains(checkedPoint)) {
                return true;
            }
        }
        return false;
    }

    public static boolean isPointInOpposingAllianceZone(Translation2d point, Collection<RectangleZone> zones) {
        return isPointInAnyZoneWithExplicitFlip(point, zones, shouldFlipForOpposingAllianceZone());
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

    public static boolean hasLineOfSightWithCircularBlocker(
        Translation2d start,
        Translation2d end,
        Translation2d blockerCenter,
        double blockerRadiusMeters,
        boolean mirrorForAlliance
    ) {
        return hasLineOfSightWithMovingCircularBlocker(
            start,
            end,
            blockerCenter,
            blockerCenter,
            blockerRadiusMeters,
            mirrorForAlliance
        );
    }

    public static boolean hasLineOfSightWithMovingCircularBlocker(
        Translation2d start,
        Translation2d end,
        Translation2d blockerStart,
        Translation2d blockerEnd,
        double blockerRadiusMeters,
        boolean mirrorForAlliance
    ) {
        Translation2d checkedStart = maybeMirror(start, mirrorForAlliance);
        Translation2d checkedEnd = maybeMirror(end, mirrorForAlliance);
        Translation2d checkedBlockerStart = maybeMirror(blockerStart, mirrorForAlliance);
        Translation2d checkedBlockerEnd = maybeMirror(blockerEnd, mirrorForAlliance);

        double minDistanceSquared = segmentToSegmentDistanceSquared(
            checkedStart,
            checkedEnd,
            checkedBlockerStart,
            checkedBlockerEnd
        );
        double effectiveRadius = Math.max(0.0, blockerRadiusMeters) + LOS_POINT_EPSILON_METERS;
        return minDistanceSquared > effectiveRadius * effectiveRadius;
    }

    public static boolean hasLineOfSightWithRectangularBlocker(
        Translation2d start,
        Translation2d end,
        RectangleZone blockerZone,
        double blockerPaddingMeters,
        boolean mirrorForAlliance
    ) {
        Translation2d checkedStart = maybeMirror(start, mirrorForAlliance);
        Translation2d checkedEnd = maybeMirror(end, mirrorForAlliance);
        Translation2d checkedCornerA = maybeMirror(blockerZone.cornerA(), mirrorForAlliance);
        Translation2d checkedCornerB = maybeMirror(blockerZone.cornerB(), mirrorForAlliance);

        double minX = Math.min(checkedCornerA.getX(), checkedCornerB.getX());
        double maxX = Math.max(checkedCornerA.getX(), checkedCornerB.getX());
        double minY = Math.min(checkedCornerA.getY(), checkedCornerB.getY());
        double maxY = Math.max(checkedCornerA.getY(), checkedCornerB.getY());

        double padding = Math.max(0.0, blockerPaddingMeters);
        minX -= padding;
        maxX += padding;
        minY -= padding;
        maxY += padding;

        return !segmentIntersectsAxisAlignedRectangle(checkedStart, checkedEnd, minX, maxX, minY, maxY);
    }

    private static Translation2d maybeMirror(Translation2d point, boolean mirrorForAlliance) {
        if (mirrorForAlliance && Constants.shouldFlipPath()) {
            return FlippingUtil.flipFieldPosition(point);
        }

        return point;
    }

    private static boolean shouldFlipForOpposingAllianceZone() {
        return !Constants.shouldFlipPath();
    }

    private static boolean isPointInAnyZoneWithExplicitFlip(
        Translation2d point,
        Collection<RectangleZone> zones,
        boolean shouldFlipPoint
    ) {
        Translation2d checkedPoint = shouldFlipPoint ? FlippingUtil.flipFieldPosition(point) : point;
        for (RectangleZone zone : zones) {
            if (zone.contains(checkedPoint)) {
                return true;
            }
        }
        return false;
    }

    private static double segmentToSegmentDistanceSquared(
        Translation2d a0,
        Translation2d a1,
        Translation2d b0,
        Translation2d b1
    ) {
        // Based on the standard closest-point solution for two finite segments.
        double ux = a1.getX() - a0.getX();
        double uy = a1.getY() - a0.getY();
        double vx = b1.getX() - b0.getX();
        double vy = b1.getY() - b0.getY();
        double wx = a0.getX() - b0.getX();
        double wy = a0.getY() - b0.getY();

        double a = ux * ux + uy * uy;
        double b = ux * vx + uy * vy;
        double c = vx * vx + vy * vy;
        double d = ux * wx + uy * wy;
        double e = vx * wx + vy * wy;
        double denominator = a * c - b * b;

        if (a <= LOS_POINT_EPSILON_METERS && c <= LOS_POINT_EPSILON_METERS) {
            double distance = a0.getDistance(b0);
            return distance * distance;
        }
        if (a <= LOS_POINT_EPSILON_METERS) {
            return pointToSegmentDistanceSquared(a0, b0, b1);
        }
        if (c <= LOS_POINT_EPSILON_METERS) {
            return pointToSegmentDistanceSquared(b0, a0, a1);
        }

        double sNumerator;
        double sDenominator = denominator;
        double tNumerator;
        double tDenominator = denominator;

        if (denominator <= LOS_POINT_EPSILON_METERS) {
            sNumerator = 0.0;
            sDenominator = 1.0;
            tNumerator = e;
            tDenominator = c;
        } else {
            sNumerator = b * e - c * d;
            tNumerator = a * e - b * d;
            if (sNumerator < 0.0) {
                sNumerator = 0.0;
                tNumerator = e;
                tDenominator = c;
            } else if (sNumerator > sDenominator) {
                sNumerator = sDenominator;
                tNumerator = e + b;
                tDenominator = c;
            }
        }

        if (tNumerator < 0.0) {
            tNumerator = 0.0;
            if (-d < 0.0) {
                sNumerator = 0.0;
            } else if (-d > a) {
                sNumerator = sDenominator;
            } else {
                sNumerator = -d;
                sDenominator = a;
            }
        } else if (tNumerator > tDenominator) {
            tNumerator = tDenominator;
            if (-d + b < 0.0) {
                sNumerator = 0.0;
            } else if (-d + b > a) {
                sNumerator = sDenominator;
            } else {
                sNumerator = -d + b;
                sDenominator = a;
            }
        }

        double sc = Math.abs(sNumerator) <= LOS_POINT_EPSILON_METERS ? 0.0 : sNumerator / sDenominator;
        double tc = Math.abs(tNumerator) <= LOS_POINT_EPSILON_METERS ? 0.0 : tNumerator / tDenominator;

        double dx = wx + sc * ux - tc * vx;
        double dy = wy + sc * uy - tc * vy;
        return dx * dx + dy * dy;
    }

    private static double pointToSegmentDistanceSquared(
        Translation2d point,
        Translation2d segmentStart,
        Translation2d segmentEnd
    ) {
        double sx = segmentEnd.getX() - segmentStart.getX();
        double sy = segmentEnd.getY() - segmentStart.getY();
        double segmentLengthSquared = sx * sx + sy * sy;

        if (segmentLengthSquared <= LOS_POINT_EPSILON_METERS) {
            double dx = point.getX() - segmentStart.getX();
            double dy = point.getY() - segmentStart.getY();
            return dx * dx + dy * dy;
        }

        double px = point.getX() - segmentStart.getX();
        double py = point.getY() - segmentStart.getY();
        double t = (px * sx + py * sy) / segmentLengthSquared;
        t = Math.max(0.0, Math.min(1.0, t));

        double closestX = segmentStart.getX() + t * sx;
        double closestY = segmentStart.getY() + t * sy;
        double dx = point.getX() - closestX;
        double dy = point.getY() - closestY;
        return dx * dx + dy * dy;
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

    private static boolean segmentIntersectsAxisAlignedRectangle(
        Translation2d start,
        Translation2d end,
        double minX,
        double maxX,
        double minY,
        double maxY
    ) {
        double x0 = start.getX();
        double y0 = start.getY();
        double x1 = end.getX();
        double y1 = end.getY();

        if (x0 >= minX && x0 <= maxX && y0 >= minY && y0 <= maxY) {
            return true;
        }
        if (x1 >= minX && x1 <= maxX && y1 >= minY && y1 <= maxY) {
            return true;
        }

        double dx = x1 - x0;
        double dy = y1 - y0;
        double tMin = 0.0;
        double tMax = 1.0;

        double[] p = {-dx, dx, -dy, dy};
        double[] q = {x0 - minX, maxX - x0, y0 - minY, maxY - y0};

        for (int i = 0; i < 4; i++) {
            double pi = p[i];
            double qi = q[i];

            if (Math.abs(pi) <= LOS_POINT_EPSILON_METERS) {
                if (qi < 0.0) {
                    return false;
                }
                continue;
            }

            double t = qi / pi;
            if (pi < 0.0) {
                if (t > tMax) {
                    return false;
                }
                tMin = Math.max(tMin, t);
            } else {
                if (t < tMin) {
                    return false;
                }
                tMax = Math.min(tMax, t);
            }
        }

        return tMax >= tMin;
    }
}
