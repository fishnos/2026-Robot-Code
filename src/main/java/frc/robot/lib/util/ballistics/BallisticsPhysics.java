package frc.robot.lib.util.ballistics;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.util.ballistics.ProjectileState.Derivatives;

public final class BallisticsPhysics {
    private static final double DEFAULT_DT = 0.002;
    private static final double MAX_SIM_TIME = 5.0;

    private BallisticsPhysics() {
    }

    public static Derivatives computeDerivatives3D(ProjectileState s, double spinRateRadPerSec) {
        double vHorizontal = Math.hypot(s.vx(), s.vy());
        double v = s.speed();

        if (v < 1e-6) {
            return new Derivatives(0.0, 0.0, 0.0, 0.0, 0.0, -BallisticsConstants.GRAVITY);
        }

        double dragForce = 0.5 * BallisticsConstants.AIR_DENSITY
            * BallisticsConstants.CROSS_SECTIONAL_AREA
            * BallisticsConstants.DRAG_COEFFICIENT * v * v;
        double dragAccel = dragForce / BallisticsConstants.BALL_MASS;

        double spinRatio = BallisticsConstants.BALL_RADIUS * Math.abs(spinRateRadPerSec) / v;
        double liftCoefficient = spinRatio / (2.0 * spinRatio + 1.0);
        liftCoefficient = Math.min(liftCoefficient, BallisticsConstants.MAX_LIFT_COEFFICIENT);

        double liftForce = 0.5 * BallisticsConstants.AIR_DENSITY
            * BallisticsConstants.CROSS_SECTIONAL_AREA * liftCoefficient * v * v;
        double liftAccel = liftForce / BallisticsConstants.BALL_MASS * Math.signum(spinRateRadPerSec);

        double vxUnit = s.vx() / v;
        double vyUnit = s.vy() / v;
        double vzUnit = s.vz() / v;

        double axDrag = -dragAccel * vxUnit;
        double ayDrag = -dragAccel * vyUnit;
        double azDrag = -dragAccel * vzUnit;

        double axLift;
        double ayLift;
        double azLift;
        if (vHorizontal > 1e-6) {
            double vHUnit = vHorizontal / v;
            double perpX = -vzUnit * (vxUnit / vHUnit);
            double perpY = -vzUnit * (vyUnit / vHUnit);
            double perpZ = vHUnit;
            double perpNorm = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);

            axLift = liftAccel * perpX / perpNorm;
            ayLift = liftAccel * perpY / perpNorm;
            azLift = liftAccel * perpZ / perpNorm;
        } else {
            axLift = 0.0;
            ayLift = 0.0;
            azLift = liftAccel;
        }

        double ax = axDrag + axLift;
        double ay = ayDrag + ayLift;
        double az = -BallisticsConstants.GRAVITY + azDrag + azLift;

        return new Derivatives(s.vx(), s.vy(), s.vz(), ax, ay, az);
    }

    public static Derivatives2D computeDerivatives2D(State2D s, double spinRateRadPerSec) {
        double v = Math.hypot(s.vx(), s.vz());
        if (v < 1e-6) {
            return new Derivatives2D(s.vx(), s.vz(), 0.0, -BallisticsConstants.GRAVITY);
        }

        double dragForce = 0.5 * BallisticsConstants.AIR_DENSITY
            * BallisticsConstants.CROSS_SECTIONAL_AREA
            * BallisticsConstants.DRAG_COEFFICIENT * v * v;
        double dragAccel = dragForce / BallisticsConstants.BALL_MASS;

        double spinRatio = BallisticsConstants.BALL_RADIUS * Math.abs(spinRateRadPerSec) / v;
        double liftCoefficient = spinRatio / (2.0 * spinRatio + 1.0);
        liftCoefficient = Math.min(liftCoefficient, BallisticsConstants.MAX_LIFT_COEFFICIENT);

        double liftForce = 0.5 * BallisticsConstants.AIR_DENSITY
            * BallisticsConstants.CROSS_SECTIONAL_AREA * liftCoefficient * v * v;
        double liftAccel = liftForce / BallisticsConstants.BALL_MASS * Math.signum(spinRateRadPerSec);

        double vxUnit = s.vx() / v;
        double vzUnit = s.vz() / v;

        double axDrag = -dragAccel * vxUnit;
        double azDrag = -dragAccel * vzUnit;

        double axLift = liftAccel * (-vzUnit);
        double azLift = liftAccel * (vxUnit);

        double ax = axDrag + axLift;
        double az = -BallisticsConstants.GRAVITY + azDrag + azLift;

        return new Derivatives2D(s.vx(), s.vz(), ax, az);
    }

    public static ProjectileState integrateStep3D(ProjectileState state, double spinRateRadPerSec, double dt) {
        Derivatives k1 = computeDerivatives3D(state, spinRateRadPerSec);
        Derivatives k2 = computeDerivatives3D(state.add(k1, dt * 0.5), spinRateRadPerSec);
        Derivatives k3 = computeDerivatives3D(state.add(k2, dt * 0.5), spinRateRadPerSec);
        Derivatives k4 = computeDerivatives3D(state.add(k3, dt), spinRateRadPerSec);

        return state.add(weighted(k1, k2, k3, k4), dt / 6.0);
    }

    public static State2D integrateStep2D(State2D state, double spinRateRadPerSec, double dt) {
        Derivatives2D k1 = computeDerivatives2D(state, spinRateRadPerSec);
        Derivatives2D k2 = computeDerivatives2D(state.add(k1, dt * 0.5), spinRateRadPerSec);
        Derivatives2D k3 = computeDerivatives2D(state.add(k2, dt * 0.5), spinRateRadPerSec);
        Derivatives2D k4 = computeDerivatives2D(state.add(k3, dt), spinRateRadPerSec);

        return state.add(weighted(k1, k2, k3, k4), dt / 6.0);
    }

    public static double estimateFlightTime(
        double distance,
        double launchAngle,
        double exitVelocity,
        double shooterHeight,
        double targetHeight,
        double spinRateRadPerSec
    ) {
        TrajectoryResult result = simulateToDistance(
            distance,
            launchAngle,
            exitVelocity,
            shooterHeight,
            targetHeight,
            spinRateRadPerSec,
            DEFAULT_DT
        );

        if (!result.reachedTarget()) {
            double horizontalVelocity = Math.max(1e-6, exitVelocity * Math.cos(launchAngle));
            return distance / horizontalVelocity;
        }

        return result.flightTime();
    }

    public static TrajectoryResult simulateToDistance(
        double distance,
        double launchAngle,
        double exitVelocity,
        double shooterHeight,
        double targetHeight,
        double spinRateRadPerSec,
        double dt
    ) {
        double vx0 = exitVelocity * Math.cos(launchAngle);
        double vz0 = exitVelocity * Math.sin(launchAngle);

        State2D state = new State2D(0.0, shooterHeight, vx0, vz0);
        State2D prev = state;
        double elapsed = 0.0;

        while (state.z() >= 0.0 && state.x() <= distance + 0.1 && elapsed < MAX_SIM_TIME) {
            prev = state;
            state = integrateStep2D(state, spinRateRadPerSec, dt);
            elapsed += dt;

            if (state.x() >= distance) {
                double t = (distance - prev.x()) / (state.x() - prev.x());
                double zAtDistance = prev.z() + t * (state.z() - prev.z());
                double timeAtDistance = (elapsed - dt) + t * dt;
                boolean reached = zAtDistance >= 0.0;
                return new TrajectoryResult(timeAtDistance, zAtDistance, distance, reached);
            }
        }

        return new TrajectoryResult(elapsed, state.z(), state.x(), false);
    }

    public static Translation3d simulateToHeight3D(
        Translation3d startPosition,
        double vx,
        double vy,
        double vz,
        double spinRateRadPerSec,
        double targetHeight,
        double dt
    ) {
        ProjectileState state = new ProjectileState(
            startPosition.getX(),
            startPosition.getY(),
            startPosition.getZ(),
            vx,
            vy,
            vz
        );
        ProjectileState prev = state;
        double elapsed = 0.0;
        double step = Math.max(1e-6, dt);

        while (state.z() >= 0.0 && elapsed < MAX_SIM_TIME) {
            prev = state;
            state = integrateStep3D(state, spinRateRadPerSec, step);
            elapsed += step;

            boolean crossedTarget =
                prev.z() >= targetHeight && state.z() <= targetHeight && state.vz() <= 0.0;
            if (crossedTarget) {
                double t = (targetHeight - prev.z()) / (state.z() - prev.z());
                double xAtTarget = prev.x() + t * (state.x() - prev.x());
                double yAtTarget = prev.y() + t * (state.y() - prev.y());
                return new Translation3d(xAtTarget, yAtTarget, targetHeight);
            }
        }

        return new Translation3d(state.x(), state.y(), state.z());
    }

    private static Derivatives weighted(Derivatives k1, Derivatives k2, Derivatives k3, Derivatives k4) {
        return new Derivatives(
            k1.dx() + 2.0 * k2.dx() + 2.0 * k3.dx() + k4.dx(),
            k1.dy() + 2.0 * k2.dy() + 2.0 * k3.dy() + k4.dy(),
            k1.dz() + 2.0 * k2.dz() + 2.0 * k3.dz() + k4.dz(),
            k1.dvx() + 2.0 * k2.dvx() + 2.0 * k3.dvx() + k4.dvx(),
            k1.dvy() + 2.0 * k2.dvy() + 2.0 * k3.dvy() + k4.dvy(),
            k1.dvz() + 2.0 * k2.dvz() + 2.0 * k3.dvz() + k4.dvz()
        );
    }

    private static Derivatives2D weighted(Derivatives2D k1, Derivatives2D k2, Derivatives2D k3, Derivatives2D k4) {
        return new Derivatives2D(
            k1.dx() + 2.0 * k2.dx() + 2.0 * k3.dx() + k4.dx(),
            k1.dz() + 2.0 * k2.dz() + 2.0 * k3.dz() + k4.dz(),
            k1.dvx() + 2.0 * k2.dvx() + 2.0 * k3.dvx() + k4.dvx(),
            k1.dvz() + 2.0 * k2.dvz() + 2.0 * k3.dvz() + k4.dvz()
        );
    }

    public record State2D(double x, double z, double vx, double vz) {
        public State2D add(Derivatives2D d, double scale) {
            return new State2D(
                x + d.dx() * scale,
                z + d.dz() * scale,
                vx + d.dvx() * scale,
                vz + d.dvz() * scale
            );
        }
    }

    public record Derivatives2D(double dx, double dz, double dvx, double dvz) {
    }
}
