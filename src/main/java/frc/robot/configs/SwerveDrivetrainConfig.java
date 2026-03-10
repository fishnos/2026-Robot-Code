package frc.robot.configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDrivetrainConfig {
    public double maxTranslationalVelocityMetersPerSec;
    public double maxTranslationalAccelerationMetersPerSecSec;
    public double maxAngularVelocityRadiansPerSec;
    public double maxAngularAccelerationRadiansPerSecSec;
    public double maxModuleVelocity;

    public double frontLeftX;
    public double frontLeftY;
    public double frontRightX;
    public double frontRightY;
    public double backLeftX;
    public double backLeftY;
    public double backRightX;
    public double backRightY;

    public double rotationCompensationCoefficient;

    public double translationControllerKP;
    public double translationControllerKI;
    public double translationControllerKD;

    public double translationToleranceMeters;
    public double translationVelocityToleranceMeters;

    public double omegaOverrideKP;
    public double omegaOverrideKI;
    public double omegaOverrideKD;
    public double rangedRotationToleranceDeg;
    public double snappedToleranceDeg;

    public double followPathTranslationKP;
    public double followPathTranslationKI;
    public double followPathTranslationKD;

    public double followPathRotationKP;
    public double followPathRotationKI;
    public double followPathRotationKD;

    public double followPathCrossTrackKP;
    public double followPathCrossTrackKI;
    public double followPathCrossTrackKD;

    public Translation2d getFrontLeftPositionMeters() {
        return new Translation2d(frontLeftX, frontLeftY);
    }

    public Translation2d getFrontRightPositionMeters() {
        return new Translation2d(frontRightX, frontRightY);
    }

    public Translation2d getBackLeftPositionMeters() {
        return new Translation2d(backLeftX, backLeftY);
    }

    public Translation2d getBackRightPositionMeters() {
        return new Translation2d(backRightX, backRightY);
    }

    public PIDController getTranslationController() {
        PIDController controller = new PIDController(translationControllerKP, translationControllerKI, translationControllerKD);
        controller.setTolerance(translationToleranceMeters, translationVelocityToleranceMeters);
        return controller;
    }
}
