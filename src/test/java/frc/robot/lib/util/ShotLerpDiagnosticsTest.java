package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.Gson;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.configs.ShooterConfig;
import frc.robot.lib.util.ballistics.BallisticsConstants;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.util.ballistics.TrajectoryResult;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class ShotLerpDiagnosticsTest {
    @Test
    // Regression: runtime visualization kinematics should match the direct wheel model at the table setpoint.
    void simLerpTable_runtimeVisualizationKinematicsMatchWheelModel() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = config.getLerpTable();
        var shootingEntries = config.getShootingLerpEntries();

        for (ShooterConfig.LerpEntry entry : shootingEntries) {
            double exitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                entry.distanceMeters,
                lerpTable,
                entry.flywheelVelocityRPS,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config)
            );
            double spinRateRadPerSec = ShotCalculator.calculateSpinRateRadPerSec(
                entry.distanceMeters,
                lerpTable,
                entry.flywheelVelocityRPS,
                measuredRps -> calculateSpinRadPerSec(measuredRps, config)
            );

            assertEquals(
                calculateShotExitVelocityMetersPerSec(entry.flywheelVelocityRPS, config),
                exitVelocity,
                1e-9
            );
            assertEquals(calculateSpinRadPerSec(entry.flywheelVelocityRPS, config), spinRateRadPerSec, 1e-9);
        }
    }

    @Test
    // Regression: pass table entries should land at target height and preserve configured TOF in the same ballistics model.
    void passLerpTable_matchesBallisticsHeightAndFlightTime() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/comp.json");
        assertTrue(config.passLerpTable != null && !config.passLerpTable.isEmpty());

        double targetHeight = 0.0;
        for (ShooterConfig.LerpEntry entry : config.passLerpTable) {
            double angleRad = Math.toRadians(entry.hoodAngleDegrees);
            double rps = entry.flywheelVelocityRPS;
            double exitVelocity = calculateShotExitVelocityMetersPerSec(rps, config);
            double spinRateRadPerSec = calculateSpinRadPerSec(rps, config);

            TrajectoryResult result = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                angleRad,
                exitVelocity,
                config.shooterPoseZ,
                targetHeight,
                spinRateRadPerSec,
                0.002
            );

            assertTrue(result.reachedTarget() || Math.abs(result.finalDistance() - entry.distanceMeters) < 0.02);
            assertTrue(Math.abs(result.finalHeight() - targetHeight) < 0.01);
            assertTrue(Math.abs(result.flightTime() - entry.flightTimeSeconds) < 0.02);
        }
    }

    @Test
    // Diagnostic: per-distance pass impact error/margin using the same runtime pass kinematics path.
    void passLerpTable_runtimeImpactMargins() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/comp.json");
        assertTrue(config.passLerpTable != null && !config.passLerpTable.isEmpty());

        InterpolatingMatrixTreeMap<Double, N3, N1> passTable = config.getPassLerpTable();
        double targetHeight = 0.0;
        double impactToleranceMeters = 0.30;
        double maxImpactErrorMeters = 0.0;

        System.out.println("PASS impact diagnostics (runtime pass path)");
        System.out.println(
            "dist_m,hood_deg,setpoint_rps,tof_cfg_s,tof_table_s,tof_direct_s,tof_runtime_s,direct_v_mps,runtime_v_mps,direct_reached,runtime_reached,direct_z_m,runtime_z_m,impact_err_m,impact_margin_m"
        );

        for (ShooterConfig.LerpEntry entry : config.passLerpTable) {
            double hoodDeg = passTable.get(entry.distanceMeters).get(0, 0);
            double setpointRps = passTable.get(entry.distanceMeters).get(1, 0);
            double tableFlightTime = passTable.get(entry.distanceMeters).get(2, 0);
            double directExitVelocity = calculateShotExitVelocityMetersPerSec(setpointRps, config);
            double directSpinRate = calculateSpinRadPerSec(setpointRps, config);

            TrajectoryResult directTrajectory = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                Math.toRadians(hoodDeg),
                directExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                directSpinRate,
                0.002
            );

            double runtimeExitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                entry.distanceMeters,
                passTable,
                setpointRps,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config)
            );
            double runtimeSpinRate = ShotCalculator.calculateSpinRateRadPerSec(
                entry.distanceMeters,
                passTable,
                setpointRps,
                measuredRps -> calculateSpinRadPerSec(measuredRps, config)
            );

            TrajectoryResult runtimeTrajectory = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                Math.toRadians(hoodDeg),
                runtimeExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                runtimeSpinRate,
                0.002
            );

            double impactErrorMeters = Math.hypot(
                runtimeTrajectory.finalDistance() - entry.distanceMeters,
                runtimeTrajectory.finalHeight() - targetHeight
            );
            double impactMarginMeters = impactToleranceMeters - impactErrorMeters;
            maxImpactErrorMeters = Math.max(maxImpactErrorMeters, impactErrorMeters);

            System.out.printf(
                Locale.US,
                "%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.0f,%.0f,%.6f,%.6f,%.4f,%.4f%n",
                entry.distanceMeters,
                hoodDeg,
                setpointRps,
                entry.flightTimeSeconds,
                tableFlightTime,
                directTrajectory.flightTime(),
                runtimeTrajectory.flightTime(),
                directExitVelocity,
                runtimeExitVelocity,
                directTrajectory.reachedTarget() ? 1.0 : 0.0,
                runtimeTrajectory.reachedTarget() ? 1.0 : 0.0,
                directTrajectory.finalHeight(),
                runtimeTrajectory.finalHeight(),
                impactErrorMeters,
                impactMarginMeters
            );
        }

        assertTrue(maxImpactErrorMeters < impactToleranceMeters);
    }

    @Test
    // Diagnostic sweep: interpolation between pass table points should stay within impact tolerance.
    void passLerpTable_interpolatedSweepImpactMargins() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/comp.json");
        assertTrue(config.passLerpTable != null && !config.passLerpTable.isEmpty());

        InterpolatingMatrixTreeMap<Double, N3, N1> passTable = config.getPassLerpTable();
        double targetHeight = 0.0;
        double impactToleranceMeters = 0.30;

        double worstDistance = 0.0;
        double worstError = 0.0;
        double worstMargin = Double.POSITIVE_INFINITY;
        double worstDistanceDirect = 0.0;
        double worstErrorDirect = 0.0;
        double worstMarginDirect = Double.POSITIVE_INFINITY;

        for (double distance = config.minPassDistFromShooterMeters; distance <= config.maxPassDistFromShooterMeters + 1e-9; distance += 0.05) {
            double hoodDeg = passTable.get(distance).get(0, 0);
            double setpointRps = passTable.get(distance).get(1, 0);

            double runtimeExitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                distance,
                passTable,
                setpointRps,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config)
            );
            double runtimeSpinRate = ShotCalculator.calculateSpinRateRadPerSec(
                distance,
                passTable,
                setpointRps,
                measuredRps -> calculateSpinRadPerSec(measuredRps, config)
            );

            TrajectoryResult runtimeResult = BallisticsPhysics.simulateToDistance(
                distance,
                Math.toRadians(hoodDeg),
                runtimeExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                runtimeSpinRate,
                0.002
            );

            double impactErrorMeters = Math.hypot(
                runtimeResult.finalDistance() - distance,
                runtimeResult.finalHeight() - targetHeight
            );
            double margin = impactToleranceMeters - impactErrorMeters;

            if (impactErrorMeters > worstError) {
                worstError = impactErrorMeters;
                worstMargin = margin;
                worstDistance = distance;
            }

            double directExitVelocity = calculateShotExitVelocityMetersPerSec(setpointRps, config);
            double directSpinRate = calculateSpinRadPerSec(setpointRps, config);
            TrajectoryResult directResult = BallisticsPhysics.simulateToDistance(
                distance,
                Math.toRadians(hoodDeg),
                directExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                directSpinRate,
                0.002
            );
            double directErrorMeters = Math.hypot(
                directResult.finalDistance() - distance,
                directResult.finalHeight() - targetHeight
            );
            double directMargin = impactToleranceMeters - directErrorMeters;
            if (directErrorMeters > worstErrorDirect) {
                worstErrorDirect = directErrorMeters;
                worstMarginDirect = directMargin;
                worstDistanceDirect = distance;
            }
        }

        System.out.printf(
            Locale.US,
            "PASS sweep worst: dist=%.3f m, error=%.4f m, margin=%.4f m%n",
            worstDistance,
            worstError,
            worstMargin
        );
        System.out.printf(
            Locale.US,
            "PASS sweep worst (direct wheel model): dist=%.3f m, error=%.4f m, margin=%.4f m%n",
            worstDistanceDirect,
            worstErrorDirect,
            worstMarginDirect
        );
        assertTrue(worstError < impactToleranceMeters);
    }

    private static ShooterConfig loadShooterConfig(String path) throws IOException {
        return new Gson().fromJson(Files.readString(Path.of(path)), ShooterConfig.class);
    }

    private static double calculateShotExitVelocityMetersPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2.0 * Math.PI * config.backRollerRadiusMeters;
        return (flywheelSurfaceVel + backRollerSurfaceVel) / 2.0;
    }

    private static double calculateSpinRadPerSec(double flywheelRps, ShooterConfig config) {
        double flywheelSurfaceVel = flywheelRps * 2.0 * Math.PI * config.flywheelRadiusMeters;
        double backRollerSurfaceVel = flywheelRps * config.backRollerGearRatio
            * 2.0 * Math.PI * config.backRollerRadiusMeters;
        double deltaV = flywheelSurfaceVel - backRollerSurfaceVel;
        return deltaV / BallisticsConstants.BALL_RADIUS;
    }
}
