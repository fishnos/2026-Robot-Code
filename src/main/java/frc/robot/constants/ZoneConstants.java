package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

public final class ZoneConstants {
    private ZoneConstants() {}

    public record RectangleZone(String name, Translation2d cornerA, Translation2d cornerB) {
        public RectangleZone {
            Objects.requireNonNull(name);
            Objects.requireNonNull(cornerA);
            Objects.requireNonNull(cornerB);
        }

        public double minX() {
            return Math.min(cornerA.getX(), cornerB.getX());
        }

        public double maxX() {
            return Math.max(cornerA.getX(), cornerB.getX());
        }

        public double minY() {
            return Math.min(cornerA.getY(), cornerB.getY());
        }

        public double maxY() {
            return Math.max(cornerA.getY(), cornerB.getY());
        }

        public boolean contains(Translation2d point) {
            double x = point.getX();
            double y = point.getY();
            return x >= minX() && x <= maxX() && y >= minY() && y <= maxY();
        }
    }

    public static final class Tower {
        private Tower() {}

        public static final RectangleZone LEFT = new RectangleZone(
            "tower_left",
            new Translation2d(1.500, 7.640),
            new Translation2d(3.456, 3.350)
        );

        public static final RectangleZone RIGHT = new RectangleZone(
            "tower_right",
            new Translation2d(1.500, 7.640),
            new Translation2d(3.456, 0.418)
        );
    }

    public static final Map<String, RectangleZone> ZONES = Map.of(
        Tower.LEFT.name(), Tower.LEFT,
        Tower.RIGHT.name(), Tower.RIGHT
    );

    public static Optional<RectangleZone> getZone(String name) {
        return Optional.ofNullable(ZONES.get(name));
    }
}
