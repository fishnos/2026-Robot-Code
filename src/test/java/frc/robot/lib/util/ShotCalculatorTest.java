package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.util.ShotCalculator.ShotData;
import frc.robot.lib.util.ballistics.BallisticsPhysics;
import frc.robot.testutil.TestLerpTableFactory;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
    private static final double EPS = 1e-9;

    @Test
    // Setpoint exit velocity should be solved so model-estimated flight time matches lerp-table flight time.
    void calculateExitVelocityMetersPerSec_matchesLerpFlightTimeWithBallisticsModel() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            45.0,
            30.0,
            0.85
        );

        double distance = 4.0;
        double hoodAngleRad = Math.toRadians(table.get(distance).get(0, 0));
        double setpointRps = table.get(distance).get(1, 0);
        double targetFlightTime = table.get(distance).get(2, 0);
        double shooterHeight = 1.0;
        double targetHeight = 2.0;
        DoubleUnaryOperator wheelRpsToExit = rps -> rps * 0.2;
        DoubleUnaryOperator wheelRpsToSpin = rps -> rps * 5.0;

        double exitVelocity = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            setpointRps,
            wheelRpsToExit,
            wheelRpsToSpin,
            shooterHeight,
            targetHeight
        );
        double spinRate = ShotCalculator.calculateSpinRateRadPerSec(
            distance,
            table,
            setpointRps,
            wheelRpsToExit,
            wheelRpsToSpin,
            shooterHeight,
            targetHeight
        );
        double estimatedFlightTime = BallisticsPhysics.estimateFlightTime(
            distance,
            hoodAngleRad,
            exitVelocity,
            shooterHeight,
            targetHeight,
            spinRate
        );

        assertEquals(targetFlightTime, estimatedFlightTime, 0.02);
    }

    @Test
    // Measured flywheel RPS should still scale solved setpoint exit velocity by measured/setpoint.
    void calculateExitVelocityMetersPerSec_scalesByMeasuredVsSetpointRps() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 3.0, 40.0, 30.0, 0.75 },
            new double[] { 5.0, 50.0, 40.0, 0.90 }
        );

        double distance = 4.0;
        double setpointRps = table.get(distance).get(1, 0);
        double full = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            setpointRps,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            1.0,
            2.0
        );
        double half = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            setpointRps * 0.5,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            1.0,
            2.0
        );

        assertEquals(0.5, half / full, 1e-6);
    }

    @Test
    // Spin should be scaled by the same exit-velocity ratio and measured/setpoint ratio as the solved shot.
    void calculateSpinRateRadPerSec_scalesWithSolvedExitVelocityRatio() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 3.0, 42.0, 30.0, 0.78 },
            new double[] { 5.0, 52.0, 40.0, 0.93 }
        );

        double distance = 4.0;
        double measuredRps = 32.0;
        double correctedExit = ShotCalculator.calculateExitVelocityMetersPerSec(
            distance,
            table,
            measuredRps,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            1.0,
            2.0
        );
        double correctedSpin = ShotCalculator.calculateSpinRateRadPerSec(
            distance,
            table,
            measuredRps,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            1.0,
            2.0
        );

        double rawMeasuredSpin = measuredRps * 5.0;
        double wheelModelMeasuredExit = measuredRps * 0.2;
        assertEquals(rawMeasuredSpin * (correctedExit / wheelModelMeasuredExit), correctedSpin, 1e-9);
    }

    @Test
    // Exit velocity should vary with hood/TOF columns even if setpoint RPS is unchanged.
    void calculateExitVelocityMetersPerSec_dependsOnHoodAndFlightTimeColumns() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.table(
            new double[] { 2.0, 35.0, 25.0, 0.50 },
            new double[] { 4.0, 60.0, 25.0, 1.10 }
        );

        double atTwoMeters = ShotCalculator.calculateExitVelocityMetersPerSec(
            2.0,
            table,
            20.0,
            rps -> rps * 0.5,
            rps -> rps * 4.0,
            1.0,
            2.0
        );
        double atFourMeters = ShotCalculator.calculateExitVelocityMetersPerSec(
            4.0,
            table,
            20.0,
            rps -> rps * 0.5,
            rps -> rps * 4.0,
            1.0,
            2.0
        );

        assertTrue(Math.abs(atTwoMeters - atFourMeters) > 1e-3);
    }

    @Test
    // Zero setpoint RPS should safely return zero exit velocity.
    void calculateExitVelocityMetersPerSec_whenSetpointRpsIsZero_returnsZero() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            30.0,
            0.0,
            1.0
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(
            1.0,
            table,
            20.0,
            rps -> rps * 0.5,
            rps -> rps * 4.0,
            1.0,
            2.0
        );
        assertEquals(0.0, actual, EPS);
    }

    @Test
    // Degenerate TOF/cos inputs should still produce finite positive velocity estimates.
    void calculateExitVelocityMetersPerSec_handlesDegenerateTofAndCosInputs() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            10.0,
            89.99,
            20.0,
            0.0
        );

        double actual = ShotCalculator.calculateExitVelocityMetersPerSec(
            2.0,
            table,
            20.0,
            rps -> rps * 0.2,
            rps -> rps * 4.0,
            1.0,
            2.0
        );

        assertTrue(Double.isFinite(actual));
        assertTrue(actual > 0.0);
    }

    @Test
    // With no robot motion, field yaw and compensated target should point directly at the goal.
    void calculate_zeroRobotVelocity_aimsDirectlyAtTarget() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(5.0, 1.0, 2.0),
            new Translation3d(1.0, 1.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(0.0, shot.targetFieldYaw().getRadians(), EPS);
        assertEquals(5.0, shot.compensatedTargetPosition().getX(), EPS);
        assertEquals(1.0, shot.compensatedTargetPosition().getY(), EPS);
        assertEquals(4.0, shot.effectiveDistance(), EPS);
    }

    @Test
    // Forward robot translation should shift compensated target backward by travel during flight.
    void calculate_withRobotTranslation_compensatesTargetPosition() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(8.0, shot.compensatedTargetPosition().getX(), EPS);
        assertEquals(0.0, shot.compensatedTargetPosition().getY(), EPS);
        assertEquals(8.0, shot.effectiveDistance(), EPS);
    }

    @Test
    // Latency term should add additional displacement compensation beyond flight-time compensation.
    void calculate_withLatency_compensatesAdditionalDisplacement() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData noLatency = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(1.0, 0.0, 0.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );
        ShotData withLatency = ShotCalculator.calculate(
            new Translation3d(10.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(1.0, 0.0, 0.0),
            table,
            0.5,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(9.0, noLatency.compensatedTargetPosition().getX(), EPS);
        assertEquals(8.5, withLatency.compensatedTargetPosition().getX(), EPS);
        assertTrue(withLatency.compensatedTargetPosition().getX() < noLatency.compensatedTargetPosition().getX());
    }

    @Test
    // Robot omega with shooter offset should contribute tangential compensation.
    void calculate_withRobotOmegaAndShooterOffset_compensatesUsingTangentialVelocity() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            45.0,
            30.0,
            1.0
        );

        ShotData shot = ShotCalculator.calculate(
            new Translation3d(0.0, 10.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(0.0, 0.0, 1.0),
            table,
            0.0,
            30.0,
            rps -> rps,
            rps -> 0.0,
            new Translation2d(1.0, 0.0),
            new Rotation2d()
        );

        assertEquals(9.0, shot.compensatedTargetPosition().getY(), EPS);
        assertTrue(shot.compensatedTargetPosition().getY() < 10.0);
    }

    @Test
    // Lower measured flywheel speed should proportionally reduce final exit velocity.
    void calculate_lowerMeasuredFlywheelRps_reducesExitVelocityProportionally() {
        InterpolatingMatrixTreeMap<Double, N3, N1> table = TestLerpTableFactory.constantRange(
            0.0,
            20.0,
            50.0,
            30.0,
            1.0
        );

        ShotData fullSpeed = ShotCalculator.calculate(
            new Translation3d(6.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            30.0,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            new Translation2d(),
            new Rotation2d()
        );
        ShotData halfSpeed = ShotCalculator.calculate(
            new Translation3d(6.0, 0.0, 2.0),
            new Translation3d(0.0, 0.0, 1.0),
            new ChassisSpeeds(),
            table,
            0.0,
            15.0,
            rps -> rps * 0.2,
            rps -> rps * 5.0,
            new Translation2d(),
            new Rotation2d()
        );

        assertEquals(0.5, halfSpeed.exitVelocity() / fullSpeed.exitVelocity(), EPS);
    }
}
