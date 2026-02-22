package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.Gson;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.configs.ShooterConfig;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.ballistics.BallisticsConstants;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.lib.util.ballistics.TrajectoryResult;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class ShotLerpDiagnosticsTest {
    @Test
    // Regression: active exit-velocity path should align with sim lerp table impact heights.
    void simLerpTable_tofDerivedExitVelocity_vsWheelModelExitVelocity() throws IOException {
        ShooterConfig config = loadShooterConfig("src/main/deploy/configs/shooter/sim.json");
        InterpolatingMatrixTreeMap<Double, N3, N1> lerpTable = config.getLerpTable();

        double targetHeight = FieldConstants.Hub.hubCenter.getZ();
        double currentMethodAbsErrorSum = 0.0;
        double tofSpeedAbsErrorSum = 0.0;
        double wheelSpeedAbsErrorSum = 0.0;

        for (ShooterConfig.LerpEntry entry : config.lerpTable) {
            double angleRad = Math.toRadians(entry.hoodAngleDegrees);
            double rps = entry.flywheelVelocityRPS;
            double spinRateRadPerSec = calculateSpinRadPerSec(rps, config);

            double tofDerivedExitVelocity = entry.distanceMeters
                / (entry.flightTimeSeconds * Math.max(0.05, Math.abs(Math.cos(angleRad))));
            double wheelModelExitVelocity = calculateShotExitVelocityMetersPerSec(rps, config);
            double currentMethodExitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
                entry.distanceMeters,
                lerpTable,
                rps,
                measuredRps -> calculateShotExitVelocityMetersPerSec(measuredRps, config)
            );

            TrajectoryResult tofResult = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                angleRad,
                tofDerivedExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                spinRateRadPerSec,
                0.002
            );

            TrajectoryResult wheelResult = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                angleRad,
                wheelModelExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                spinRateRadPerSec,
                0.002
            );

            TrajectoryResult currentResult = BallisticsPhysics.simulateToDistance(
                entry.distanceMeters,
                angleRad,
                currentMethodExitVelocity,
                config.shooterPoseZ,
                targetHeight,
                spinRateRadPerSec,
                0.002
            );

            double currentHeightError = currentResult.finalHeight() - targetHeight;
            double tofHeightError = tofResult.finalHeight() - targetHeight;
            double wheelHeightError = wheelResult.finalHeight() - targetHeight;
            currentMethodAbsErrorSum += Math.abs(currentHeightError);
            tofSpeedAbsErrorSum += Math.abs(tofHeightError);
            wheelSpeedAbsErrorSum += Math.abs(wheelHeightError);
        }

        assertTrue(currentMethodAbsErrorSum / config.lerpTable.size() < 0.10);
        assertTrue(tofSpeedAbsErrorSum > currentMethodAbsErrorSum * 10.0);
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
