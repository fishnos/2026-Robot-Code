package frc.robot.lib.util.ballistics;

public record ProjectileState(
    double x,
    double y,
    double z,
    double vx,
    double vy,
    double vz
) {
    public static ProjectileState of2D(double x, double z, double vx, double vz) {
        return new ProjectileState(x, 0.0, z, vx, 0.0, vz);
    }

    public ProjectileState add(Derivatives d, double scale) {
        return new ProjectileState(
            x + d.dx * scale,
            y + d.dy * scale,
            z + d.dz * scale,
            vx + d.dvx * scale,
            vy + d.dvy * scale,
            vz + d.dvz * scale
        );
    }

    public double speed() {
        return Math.sqrt(vx * vx + vy * vy + vz * vz);
    }

    public record Derivatives(
        double dx,
        double dy,
        double dz,
        double dvx,
        double dvy,
        double dvz
    ) {
    }
}
