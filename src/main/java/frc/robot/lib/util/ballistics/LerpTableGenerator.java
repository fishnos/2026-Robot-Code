package frc.robot.lib.util.ballistics;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import frc.robot.configs.ShooterConfig;

public final class LerpTableGenerator {
    private static final double SIMULATION_DT = 0.002;

    private static final double HUB_HEIGHT_METERS = 1.8034; // 71 inches
    private static final double WAYPOINT_HEIGHT_METERS = 2.1844; // 86 inches
    private static final double WAYPOINT_OFFSET_METERS = 0.381; // 15 inches

    private static final double[] DEFAULT_DISTANCES_INCHES = {
        35, 48, 60, 72, 96, 120, 144, 180, 216, 252, 300, 360, 420
    };

    private LerpTableGenerator() {
    }

    public static void main(String[] args) throws IOException {
        if (args.length < 1) {
            System.err.println("Usage: LerpTableGenerator <configPath>");
            System.exit(1);
        }

        ShooterConfig config = loadConfig(Path.of(args[0]));
        List<LerpEntry> entries = generate(config);
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        System.out.println(gson.toJson(entries));
    }

    private static ShooterConfig loadConfig(Path path) throws IOException {
        String json = Files.readString(path);
        return new Gson().fromJson(json, ShooterConfig.class);
    }

    public static List<LerpEntry> generate(ShooterConfig config) {
        double shooterHeight = config.shooterPoseZ;
        double minAngle = config.hoodMinAngleRotations * 2.0 * Math.PI;
        double maxAngle = config.hoodMaxAngleRotations * 2.0 * Math.PI;

        List<LerpEntry> results = new ArrayList<>();
        for (double inches : DEFAULT_DISTANCES_INCHES) {
            double distanceMeters = inches * 0.0254;
            double waypointDistance = Math.max(0.0, distanceMeters - WAYPOINT_OFFSET_METERS);

            Solution solution = solve(distanceMeters, waypointDistance, shooterHeight, minAngle, maxAngle, config);
            results.add(new LerpEntry(distanceMeters, solution.hoodAngleRotations, solution.flywheelRps));
        }
        return results;
    }

    private static Solution solve(
        double targetDistance,
        double waypointDistance,
        double shooterHeight,
        double minAngle,
        double maxAngle,
        ShooterConfig config
    ) {
        Solution best = new Solution(0, 0, Double.POSITIVE_INFINITY);
        for (double angle = minAngle; angle <= maxAngle; angle += Math.toRadians(2.0)) {
            for (double rps = 5.0; rps <= 80.0; rps += 2.0) {
                ErrorSample sample = simulate(angle, rps, shooterHeight, waypointDistance, targetDistance, config);
                if (!sample.valid) {
                    continue;
                }
                double error = sq(sample.waypointHeight - WAYPOINT_HEIGHT_METERS)
                    + sq(sample.targetHeight - HUB_HEIGHT_METERS);
                if (error < best.error) {
                    best = new Solution(angle / (2.0 * Math.PI), rps, error);
                }
            }
        }

        double bestAngle = best.hoodAngleRotations * 2.0 * Math.PI;
        for (double angle = bestAngle - Math.toRadians(4.0); angle <= bestAngle + Math.toRadians(4.0); angle += Math.toRadians(0.5)) {
            if (angle < minAngle || angle > maxAngle) {
                continue;
            }
            for (double rps = best.flywheelRps - 6.0; rps <= best.flywheelRps + 6.0; rps += 0.5) {
                if (rps <= 0.0) {
                    continue;
                }
                ErrorSample sample = simulate(angle, rps, shooterHeight, waypointDistance, targetDistance, config);
                if (!sample.valid) {
                    continue;
                }
                double error = sq(sample.waypointHeight - WAYPOINT_HEIGHT_METERS)
                    + sq(sample.targetHeight - HUB_HEIGHT_METERS);
                if (error < best.error) {
                    best = new Solution(angle / (2.0 * Math.PI), rps, error);
                }
            }
        }

        return best;
    }

    private static ErrorSample simulate(
        double angle,
        double flywheelRps,
        double shooterHeight,
        double waypointDistance,
        double targetDistance,
        ShooterConfig config
    ) {
        double exitVelocity = calculateExitVelocity(flywheelRps, config);
        double spinRateRadPerSec = calculateSpinRadPerSec(flywheelRps, config);

        BallisticsPhysics.State2D state = new BallisticsPhysics.State2D(
            0.0,
            shooterHeight,
            exitVelocity * Math.cos(angle),
            exitVelocity * Math.sin(angle)
        );

        Double waypointHeight = null;
        Double targetHeight = null;

        double elapsed = 0.0;
        while (state.z() >= 0.0 && state.x() <= targetDistance + 0.1 && elapsed < 5.0) {
            BallisticsPhysics.State2D prev = state;
            state = BallisticsPhysics.integrateStep2D(state, spinRateRadPerSec, SIMULATION_DT);
            elapsed += SIMULATION_DT;

            if (waypointHeight == null && state.x() >= waypointDistance) {
                waypointHeight = interpolateHeight(prev, state, waypointDistance);
            }
            if (targetHeight == null && state.x() >= targetDistance) {
                targetHeight = interpolateHeight(prev, state, targetDistance);
                break;
            }
        }

        if (waypointHeight == null || targetHeight == null) {
            return new ErrorSample(false, 0, 0);
        }
        return new ErrorSample(true, waypointHeight, targetHeight);
    }

    private static double interpolateHeight(BallisticsPhysics.State2D prev, BallisticsPhysics.State2D next, double targetX) {
        if (Math.abs(next.x() - prev.x()) < 1e-6) {
            return next.z();
        }
        double t = (targetX - prev.x()) / (next.x() - prev.x());
        return prev.z() + t * (next.z() - prev.z());
    }

    private static double calculateExitVelocity(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    private static double calculateSpinRadPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2 * Math.PI * config.backRollerRadiusMeters;
        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        return deltaV / BallisticsConstants.BALL_RADIUS;
    }

    private static double sq(double value) {
        return value * value;
    }

    private static final class ErrorSample {
        final boolean valid;
        final double waypointHeight;
        final double targetHeight;

        ErrorSample(boolean valid, double waypointHeight, double targetHeight) {
            this.valid = valid;
            this.waypointHeight = waypointHeight;
            this.targetHeight = targetHeight;
        }
    }

    private static final class Solution {
        final double hoodAngleRotations;
        final double flywheelRps;
        final double error;

        Solution(double hoodAngleRotations, double flywheelRps, double error) {
            this.hoodAngleRotations = hoodAngleRotations;
            this.flywheelRps = flywheelRps;
            this.error = error;
        }
    }

    public static final class LerpEntry {
        public final double distanceMeters;
        public final double hoodAngleRotations;
        public final double flywheelVelocityRPS;

        LerpEntry(double distanceMeters, double hoodAngleRotations, double flywheelVelocityRPS) {
            this.distanceMeters = distanceMeters;
            this.hoodAngleRotations = hoodAngleRotations;
            this.flywheelVelocityRPS = flywheelVelocityRPS;
        }
    }
}
