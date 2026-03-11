package frc.robot.lib.util.ballistics;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.configs.ShooterConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates pass lerp-table entries by solving trajectories against the shared ballistics model.
 *
 * Usage:
 * PassLerpTableGenerator <configPath> [minDistanceMeters] [maxDistanceMeters] [intervalMeters] [targetHeightMeters]
 */
public final class PassLerpTableGenerator {
    private static final double SIM_DT_SECONDS = 0.002;

    private static final double DEFAULT_MIN_DISTANCE_METERS = 1.0;
    private static final double DEFAULT_MAX_DISTANCE_METERS = 17.0;
    private static final double DEFAULT_INTERVAL_METERS = 1.2;
    private static final double DEFAULT_TARGET_HEIGHT_METERS = 0.0;

    private static final double MIN_RPS = 5.0;
    private static final double MAX_RPS = 140.0;
    private static final int RPS_BISECTION_ITERS = 42;
    private static final double HEIGHT_TOLERANCE_METERS = 1e-4;
    private static final double EXACT_SOLUTION_TOLERANCE_METERS = 1e-3;
    private static final double ANGLE_STEP_DEG = 0.1;

    private PassLerpTableGenerator() {}

    public static void main(String[] args) throws IOException {
        if (args.length < 1) {
            System.err.println(
                "Usage: PassLerpTableGenerator <configPath> [minDistanceMeters] [maxDistanceMeters] [intervalMeters] [targetHeightMeters]"
            );
            System.exit(1);
        }

        Path configPath = Path.of(args[0]);
        double minDistance = args.length >= 2 ? Double.parseDouble(args[1]) : DEFAULT_MIN_DISTANCE_METERS;
        double maxDistance = args.length >= 3 ? Double.parseDouble(args[2]) : DEFAULT_MAX_DISTANCE_METERS;
        double interval = args.length >= 4 ? Double.parseDouble(args[3]) : DEFAULT_INTERVAL_METERS;
        double targetHeight = args.length >= 5 ? Double.parseDouble(args[4]) : DEFAULT_TARGET_HEIGHT_METERS;

        ShooterConfig config = loadConfig(configPath);
        List<ShooterConfig.LerpEntry> entries = generate(config, minDistance, maxDistance, interval, targetHeight);

        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        System.out.println(gson.toJson(entries));
    }

    public static List<ShooterConfig.LerpEntry> generate(
        ShooterConfig config,
        double minDistanceMeters,
        double maxDistanceMeters,
        double intervalMeters,
        double targetHeightMeters
    ) {
        List<Double> distances = buildDistances(minDistanceMeters, maxDistanceMeters, intervalMeters);

        double minAngleDeg = Math.min(config.hoodMinAngleDegrees, config.hoodMaxAngleDegrees);
        double maxAngleDeg = Math.max(config.hoodMinAngleDegrees, config.hoodMaxAngleDegrees);
        InterpolatingMatrixTreeMap<Double, N3, N1> shootingTable = config.getLerpTable();

        List<ShooterConfig.LerpEntry> entries = new ArrayList<>();
        Solution previousSolution = null;
        for (double distanceMeters : distances) {
            double preferredHoodAngleDeg = getPreferredPassHoodAngleDeg(distanceMeters, config, shootingTable);
            Solution solution = solveDistance(
                distanceMeters,
                targetHeightMeters,
                config.shooterPoseZ,
                minAngleDeg,
                maxAngleDeg,
                preferredHoodAngleDeg,
                previousSolution,
                config
            );
            previousSolution = solution;

            ShooterConfig.LerpEntry entry = new ShooterConfig.LerpEntry();
            entry.distanceMeters = round(distanceMeters, 3);
            entry.hoodAngleDegrees = round(solution.hoodAngleDeg(), 3);
            entry.flywheelVelocityRPS = round(solution.flywheelRps(), 3);
            entry.flightTimeSeconds = round(solution.flightTimeSeconds(), 6);
            entries.add(entry);
        }
        return entries;
    }

    private static List<Double> buildDistances(double minDistanceMeters, double maxDistanceMeters, double intervalMeters) {
        List<Double> distances = new ArrayList<>();
        if (intervalMeters <= 0.0) {
            distances.add(minDistanceMeters);
            if (maxDistanceMeters > minDistanceMeters) {
                distances.add(maxDistanceMeters);
            }
            return distances;
        }

        for (double d = minDistanceMeters; d <= maxDistanceMeters + 1e-9; d += intervalMeters) {
            distances.add(round(d, 6));
        }
        double last = distances.get(distances.size() - 1);
        if (Math.abs(last - maxDistanceMeters) > 1e-6) {
            distances.add(round(maxDistanceMeters, 6));
        }
        return distances;
    }

    private static Solution solveDistance(
        double distanceMeters,
        double targetHeightMeters,
        double shooterHeightMeters,
        double minAngleDeg,
        double maxAngleDeg,
        double preferredHoodAngleDeg,
        Solution previousSolution,
        ShooterConfig config
    ) {
        Solution bestExact = null;
        Solution bestFallback = null;
        double bestFallbackHeightErrorMeters = Double.POSITIVE_INFINITY;

        for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg + 1e-9; angleDeg += ANGLE_STEP_DEG) {
            double angleRad = Math.toRadians(angleDeg);
            Solution forAngle = solveForAngle(distanceMeters, targetHeightMeters, shooterHeightMeters, angleRad, config);
            if (forAngle == null) {
                continue;
            }

            double heightErrorMeters = Math.abs(forAngle.finalHeightMeters() - targetHeightMeters);
            if (heightErrorMeters <= EXACT_SOLUTION_TOLERANCE_METERS) {
                if (bestExact == null
                    || isBetterExactSolution(forAngle, bestExact, preferredHoodAngleDeg, previousSolution)) {
                    bestExact = forAngle;
                }
                continue;
            }

            // Fallback: if no exact solution exists for the distance, use closest hit then lowest TOF.
            if (heightErrorMeters < bestFallbackHeightErrorMeters - 1e-8) {
                bestFallbackHeightErrorMeters = heightErrorMeters;
                bestFallback = forAngle;
            } else if (Math.abs(heightErrorMeters - bestFallbackHeightErrorMeters) <= 1e-8
                && bestFallback != null
                && isBetterExactSolution(forAngle, bestFallback, preferredHoodAngleDeg, previousSolution)) {
                bestFallback = forAngle;
            }
        }

        if (bestExact != null) {
            return bestExact;
        }
        if (bestFallback == null) {
            throw new IllegalStateException("No ballistic pass solution for distance " + distanceMeters + " meters");
        }
        return bestFallback;
    }

    private static boolean isBetterExactSolution(
        Solution candidate,
        Solution incumbent,
        double preferredHoodAngleDeg,
        Solution previousSolution
    ) {
        if (Double.isFinite(preferredHoodAngleDeg)) {
            double candidatePreferredError = Math.abs(candidate.hoodAngleDeg() - preferredHoodAngleDeg);
            double incumbentPreferredError = Math.abs(incumbent.hoodAngleDeg() - preferredHoodAngleDeg);
            if (candidatePreferredError < incumbentPreferredError - 1e-8) {
                return true;
            }
            if (candidatePreferredError > incumbentPreferredError + 1e-8) {
                return false;
            }
        }

        if (previousSolution != null) {
            double candidateContinuityCost = continuityCost(candidate, previousSolution);
            double incumbentContinuityCost = continuityCost(incumbent, previousSolution);
            if (candidateContinuityCost < incumbentContinuityCost - 1e-8) {
                return true;
            }
            if (candidateContinuityCost > incumbentContinuityCost + 1e-8) {
                return false;
            }
            // With equal continuity, prefer less extreme hood movement.
            if (candidate.hoodAngleDeg() < incumbent.hoodAngleDeg() - 1e-8) {
                return true;
            }
            if (candidate.hoodAngleDeg() > incumbent.hoodAngleDeg() + 1e-8) {
                return false;
            }
        }
        if (candidate.flywheelRps() < incumbent.flywheelRps() - 1e-8) {
            return true;
        }
        if (candidate.flywheelRps() > incumbent.flywheelRps() + 1e-8) {
            return false;
        }
        if (candidate.flightTimeSeconds() < incumbent.flightTimeSeconds() - 1e-8) {
            return true;
        }
        return false;
    }

    private static double continuityCost(Solution candidate, Solution previousSolution) {
        double angleDeltaDeg = Math.abs(candidate.hoodAngleDeg() - previousSolution.hoodAngleDeg());
        double rpsDelta = Math.abs(candidate.flywheelRps() - previousSolution.flywheelRps());
        return angleDeltaDeg + (0.15 * rpsDelta);
    }

    private static double getPreferredPassHoodAngleDeg(
        double distanceMeters,
        ShooterConfig config,
        InterpolatingMatrixTreeMap<Double, N3, N1> shootingTable
    ) {
        if (config.getShootingLerpEntries() == null || config.getShootingLerpEntries().isEmpty()) {
            return Double.NaN;
        }

        double minShot = config.minShotDistFromShooterMeters;
        double maxShot = config.maxShotDistFromShooterMeters;
        if (!Double.isFinite(minShot) || !Double.isFinite(maxShot) || maxShot < minShot) {
            return Double.NaN;
        }

        double clampedDistance = Math.max(minShot, Math.min(maxShot, distanceMeters));
        return shootingTable.get(clampedDistance).get(0, 0);
    }

    private static Solution solveForAngle(
        double distanceMeters,
        double targetHeightMeters,
        double shooterHeightMeters,
        double hoodAngleRad,
        ShooterConfig config
    ) {
        TrajectoryEval lowEval = evaluate(distanceMeters, targetHeightMeters, shooterHeightMeters, hoodAngleRad, MIN_RPS, config);
        TrajectoryEval highEval = evaluate(distanceMeters, targetHeightMeters, shooterHeightMeters, hoodAngleRad, MAX_RPS, config);

        double lowError = lowEval.solveError();
        double highError = highEval.solveError();

        // Need a sign change in height error to bracket the root.
        if (Math.signum(lowError) == Math.signum(highError) && Math.abs(lowError) > HEIGHT_TOLERANCE_METERS
            && Math.abs(highError) > HEIGHT_TOLERANCE_METERS) {
            return null;
        }

        double lo = MIN_RPS;
        double hi = MAX_RPS;
        TrajectoryEval loEval = lowEval;
        TrajectoryEval hiEval = highEval;

        for (int i = 0; i < RPS_BISECTION_ITERS; i++) {
            double mid = (lo + hi) * 0.5;
            TrajectoryEval midEval = evaluate(distanceMeters, targetHeightMeters, shooterHeightMeters, hoodAngleRad, mid, config);
            double midError = midEval.solveError();
            if (Math.abs(midError) <= HEIGHT_TOLERANCE_METERS) {
                if (!midEval.trajectoryResult().reachedTarget()) {
                    return null;
                }
                return new Solution(
                    hoodAngleRad,
                    mid,
                    midEval.trajectoryResult().flightTime(),
                    midEval.trajectoryResult().finalHeight()
                );
            }

            double loError = loEval.solveError();
            if (Math.signum(loError) == Math.signum(midError)) {
                lo = mid;
                loEval = midEval;
            } else {
                hi = mid;
                hiEval = midEval;
            }
        }

        // Use the better endpoint after bisection iterations complete.
        TrajectoryEval finalEval = Math.abs(loEval.solveError()) <= Math.abs(hiEval.solveError()) ? loEval : hiEval;
        double finalRps = finalEval == loEval ? lo : hi;
        if (!finalEval.trajectoryResult().reachedTarget()) {
            return null;
        }
        return new Solution(
            hoodAngleRad,
            finalRps,
            finalEval.trajectoryResult().flightTime(),
            finalEval.trajectoryResult().finalHeight()
        );
    }

    private static TrajectoryEval evaluate(
        double distanceMeters,
        double targetHeightMeters,
        double shooterHeightMeters,
        double hoodAngleRad,
        double flywheelRps,
        ShooterConfig config
    ) {
        double exitVelocityMetersPerSec = calculateExitVelocity(flywheelRps, config);
        double spinRateRadPerSec = calculateSpinRadPerSec(flywheelRps, config);
        var result = BallisticsPhysics.simulateToDistance(
            distanceMeters,
            hoodAngleRad,
            exitVelocityMetersPerSec,
            shooterHeightMeters,
            targetHeightMeters,
            spinRateRadPerSec,
            SIM_DT_SECONDS
        );
        double solveError = result.reachedTarget()
            ? (result.finalHeight() - targetHeightMeters)
            : -(distanceMeters - result.finalDistance());
        return new TrajectoryEval(result, solveError);
    }

    private static ShooterConfig loadConfig(Path configPath) throws IOException {
        return new Gson().fromJson(Files.readString(configPath), ShooterConfig.class);
    }

    private static double calculateExitVelocity(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel =
            flywheelRps * config.backRollerGearRatio * 2.0 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    private static double calculateSpinRadPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel =
            flywheelRps * config.backRollerGearRatio * 2.0 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel - backRollerSurfaceVel) / BallisticsConstants.BALL_RADIUS;
    }

    private static double round(double value, int decimals) {
        double factor = Math.pow(10.0, decimals);
        return Math.round(value * factor) / factor;
    }

    private record TrajectoryEval(
        TrajectoryResult trajectoryResult,
        double solveError
    ) {}

    private record Solution(
        double hoodAngleRad,
        double flywheelRps,
        double flightTimeSeconds,
        double finalHeightMeters
    ) {
        double hoodAngleDeg() {
            return Math.toDegrees(hoodAngleRad);
        }
    }
}
