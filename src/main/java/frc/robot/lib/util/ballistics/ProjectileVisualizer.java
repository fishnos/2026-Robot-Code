package frc.robot.lib.util.ballistics;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Visualizes one or more projectiles using physics simulation with drag and Magnus effect.
 * Uses numerical integration to simulate the trajectory of the 2026 "Rebuilt" Fuel game piece.
 */
public class ProjectileVisualizer extends SubsystemBase {
    private static final double SIMULATION_DT = 0.001;

    private static final ProjectileVisualizer INSTANCE = new ProjectileVisualizer();

    public static ProjectileVisualizer getInstance() {
        return INSTANCE;
    }

    /**
     * Add a projectile with constant parameters (no spin/Magnus effect).
     */
    public static int addProjectile(
            double robotVx,
            double robotVy,
            double launchVelocity,
            Pose3d launchPose,
            double finalZ) {
        return INSTANCE.addProjectileInternal(
                () -> robotVx,
                () -> robotVy,
                () -> launchVelocity,
                () -> launchPose,
                () -> finalZ,
                () -> 0.0);
    }

    /**
     * Add a projectile with constant parameters including spin rate for Magnus effect.
     * @param spinRateRPM Spin rate in RPM (positive = backspin for upward lift)
     */
    public static int addProjectile(
            double robotVx,
            double robotVy,
            double launchVelocity,
            Pose3d launchPose,
            double finalZ,
            double spinRateRPM) {
        return INSTANCE.addProjectileInternal(
                () -> robotVx,
                () -> robotVy,
                () -> launchVelocity,
                () -> launchPose,
                () -> finalZ,
                () -> spinRateRPM);
    }

    /**
     * Add a projectile with supplier parameters (no spin/Magnus effect).
     */
    public static int addProjectile(
            Supplier<Double> robotVxSupplier,
            Supplier<Double> robotVySupplier,
            Supplier<Double> launchVelocitySupplier,
            Supplier<Pose3d> launchPoseSupplier,
            Supplier<Double> finalZSupplier) {
        return INSTANCE.addProjectileInternal(
                robotVxSupplier,
                robotVySupplier,
                launchVelocitySupplier,
                launchPoseSupplier,
                finalZSupplier,
                () -> 0.0);
    }

    /**
     * Add a projectile with supplier parameters including spin rate for Magnus effect.
     * @param spinRateRPMSupplier Spin rate in RPM (positive = backspin for upward lift)
     */
    public static int addProjectile(
            Supplier<Double> robotVxSupplier,
            Supplier<Double> robotVySupplier,
            Supplier<Double> launchVelocitySupplier,
            Supplier<Pose3d> launchPoseSupplier,
            Supplier<Double> finalZSupplier,
            Supplier<Double> spinRateRPMSupplier) {
        return INSTANCE.addProjectileInternal(
                robotVxSupplier,
                robotVySupplier,
                launchVelocitySupplier,
                launchPoseSupplier,
                finalZSupplier,
                spinRateRPMSupplier);
    }

    public static void clearProjectiles() {
        INSTANCE.clearProjectilesInternal();
    }

    private final ArrayList<ProjectileInstance> projectiles = new ArrayList<>();
    private final ArrayList<Translation3d> translations = new ArrayList<>(32);
    private int nextId = 0;

    private ProjectileVisualizer() {
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        Iterator<ProjectileInstance> iterator = projectiles.iterator();

        while (iterator.hasNext()) {
            ProjectileInstance projectile = iterator.next();
            if (!projectile.initialized) {
                projectile.initialize(now);
            }

            projectile.update(now);
            translations.set(projectile.id, projectile.currentPosition);

            if (projectile.isFinished()) {
                projectile.clearLogs();
                translations.set(projectile.id, new Translation3d());
                iterator.remove();
            }
        }

        Logger.recordOutput("ProjectileVisualizer/Translations", translations.toArray(new Translation3d[0]));
        Logger.recordOutput("ProjectileVisualizer/ActiveCount", projectiles.size());
    }

    private int addProjectileInternal(
            Supplier<Double> robotVxSupplier,
            Supplier<Double> robotVySupplier,
            Supplier<Double> launchVelocitySupplier,
            Supplier<Pose3d> launchPoseSupplier,
            Supplier<Double> finalZSupplier,
            Supplier<Double> spinRateRPMSupplier) {
        int id = nextId++;
        while (translations.size() <= id) {
            translations.add(new Translation3d());
        }
        projectiles.add(new ProjectileInstance(
                id,
                robotVxSupplier,
                robotVySupplier,
                launchVelocitySupplier,
                launchPoseSupplier,
                finalZSupplier,
                spinRateRPMSupplier));
        return id;
    }

    private void clearProjectilesInternal() {
        for (ProjectileInstance projectile : projectiles) {
            projectile.clearLogs();
        }
        projectiles.clear();
        translations.clear();
        nextId = 0;
        Logger.recordOutput("ProjectileVisualizer/Translations", new Translation3d[0]);
        Logger.recordOutput("ProjectileVisualizer/ActiveCount", 0);
    }

    private static final class ProjectileInstance {
        private final int id;
        private final Supplier<Double> robotVxSupplier;
        private final Supplier<Double> robotVySupplier;
        private final Supplier<Double> launchVelocitySupplier;
        private final Supplier<Pose3d> launchPoseSupplier;
        private final Supplier<Double> finalZSupplier;
        private final Supplier<Double> spinRateRPMSupplier;

        private boolean initialized = false;
        private Translation3d currentPosition = new Translation3d();
        private Pose3d initialPose = new Pose3d();
        private double startTime;
        private double lastUpdateTime;
        private double launchAngle;
        private double launchYaw;
        private double finalZ;
        private double spinRateRadPerSec;

        private ProjectileState state;

        private ProjectileInstance(
                int id,
                Supplier<Double> robotVxSupplier,
                Supplier<Double> robotVySupplier,
                Supplier<Double> launchVelocitySupplier,
                Supplier<Pose3d> launchPoseSupplier,
                Supplier<Double> finalZSupplier,
                Supplier<Double> spinRateRPMSupplier) {
            this.id = id;
            this.robotVxSupplier = robotVxSupplier;
            this.robotVySupplier = robotVySupplier;
            this.launchVelocitySupplier = launchVelocitySupplier;
            this.launchPoseSupplier = launchPoseSupplier;
            this.finalZSupplier = finalZSupplier;
            this.spinRateRPMSupplier = spinRateRPMSupplier;
        }

        private void initialize(double now) {
            startTime = now;
            lastUpdateTime = now;
            initializeState();
            currentPosition = initialPose.getTranslation();
            Logger.recordOutput(logKey("InitialPose"), initialPose);
            Logger.recordOutput(logKey("LaunchAngleDegrees"), Math.toDegrees(launchAngle));
            Logger.recordOutput(logKey("LaunchVelocity"), launchVelocitySupplier.get());
            Logger.recordOutput(logKey("SpinRateRPM"), spinRateRPMSupplier.get());
            initialized = true;
        }

        private void initializeState() {
            Pose3d pose = launchPoseSupplier.get();
            if (pose == null) {
                pose = new Pose3d();
            }
            initialPose = new Pose3d(
                    pose.getX(),
                    pose.getY(),
                    pose.getZ(),
                    new Rotation3d(
                            pose.getRotation().getX(),
                            pose.getRotation().getY(),
                            pose.getRotation().getZ()));

            Rotation3d rotation = initialPose.getRotation();
            launchAngle = rotation.getY();
            launchYaw = rotation.getZ();

            double launchVelocity = launchVelocitySupplier.get();
            double launchVxComponent = launchVelocity * Math.cos(launchAngle) * Math.cos(launchYaw);
            double launchVyComponent = launchVelocity * Math.cos(launchAngle) * Math.sin(launchYaw);
            double launchVzComponent = launchVelocity * Math.sin(launchAngle);

            double vx0 = robotVxSupplier.get() + launchVxComponent;
            double vy0 = robotVySupplier.get() + launchVyComponent;
            double vz0 = launchVzComponent;

            finalZ = finalZSupplier.get();

            spinRateRadPerSec = spinRateRPMSupplier.get() * 2.0 * Math.PI / 60.0;

            state = new ProjectileState(
                    initialPose.getX(),
                    initialPose.getY(),
                    initialPose.getZ(),
                    vx0,
                    vy0,
                    vz0);
        }

        private void update(double now) {
            double elapsedSinceLastUpdate = now - lastUpdateTime;
            lastUpdateTime = now;

            double remainingTime = elapsedSinceLastUpdate;
            while (remainingTime > 0 && state.z() >= 0) {
                double dt = Math.min(SIMULATION_DT, remainingTime);
                state = BallisticsPhysics.integrateStep3D(state, spinRateRadPerSec, dt);
                remainingTime -= dt;
            }

            double z = Math.max(0.0, state.z());
            state = new ProjectileState(state.x(), state.y(), z, state.vx(), state.vy(), state.vz());

            currentPosition = new Translation3d(state.x(), state.y(), state.z());
            Pose3d currentPose = new Pose3d(currentPosition, initialPose.getRotation());

            double elapsedTotal = now - startTime;
            Logger.recordOutput(logKey("CurrentPose"), currentPose);
            Logger.recordOutput(logKey("CurrentPosition"), currentPosition);
            Logger.recordOutput(logKey("ElapsedTime"), elapsedTotal);
            Logger.recordOutput(logKey("Height"), state.z());
            Logger.recordOutput(logKey("VelocityMagnitude"), state.speed());
        }

        private void clearLogs() {
            Logger.recordOutput(logKey("CurrentPose"), new Pose3d());
            Logger.recordOutput(logKey("CurrentPosition"), new Translation3d());
            Logger.recordOutput(logKey("ElapsedTime"), 0.0);
            Logger.recordOutput(logKey("Height"), 0.0);
            Logger.recordOutput(logKey("VelocityMagnitude"), 0.0);
        }

        private boolean isFinished() {
            return state.vz() <= 0 && state.z() <= finalZ;
        }

        private String logKey(String name) {
            return "ProjectileVisualizer/Projectile" + "/" + name;
        }
    }
}
