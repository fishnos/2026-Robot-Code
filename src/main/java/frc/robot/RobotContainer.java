package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.AlignmentConstants;
// import frc.robot.commands.autos.tower.ScoreL1;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.ballistics.ProjectileVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Vision vision = Vision.getInstance();
    @SuppressWarnings("unused")
    private final ProjectileVisualizer projectileVisualizer = ProjectileVisualizer.getInstance();


    private RobotContainer() {
        registerEventTriggers();

        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // Configure teleop input suppliers for SwerveDrive FSM
        // Using normalized inputs (-1 to 1) with deadband applied
        swerveDrive.setTeleopInputSuppliers(
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getLeftX(), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND)
        );

        // Set default state to TELEOP
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
        
        // Set default superstructure state to HOME
        superstructure.setDesiredState(Superstructure.DesiredState.HOME);


        configureBindings();
    }

    private void registerEventTriggers() {
        FollowPath.registerEventTrigger("test1", new InstantCommand(() -> {
            Logger.recordOutput("In a command", true);
        }));
        FollowPath.registerEventTrigger("test2", () -> {
            System.out.println("As a runnable");
        });
    }

    private void configureBindings() {
        // Path toClimb = new Path(new Waypoint(1,3, new Rotation2d(0)));
        // xboxDriver.getXButton().onTrue(
        //     new InstantCommand(() -> currentPath = toClimb).andThen(
        //         new InstantCommand(() -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)).andThen(
        //             new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE)).andThen(
        //                 new InstantCommand(() -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP))
        //             )
        //         )
        // );

        // xboxDriver.getXButton().onFalse(new InstantCommand(() -> robotState.resetPose(new Pose2d(0,0, new Rotation2d(0)))));

        // xboxDriver.getAButton().onTrue(new InstantCommand(() -> {
        //         ProjectileVisualizer.addProjectile(
        //             0, // vx
        //             0, // vy
        //             10, // exitVelocity
        //             new Pose3d(0, 0, 2, new Rotation3d(0, Math.PI/4, 0)), // position & rotation
        //             3 // hub height
        //         );
        //     })
        // );

        xboxDriver.getAButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(DesiredState.SHOOTING))
        );
        xboxDriver.getBButton().onTrue(
            new InstantCommand(() -> superstructure.setDesiredState(DesiredState.TRACKING))
        );

        // xboxDriver.getAButton().onTrue(
        //     new ConditionalCommand(
        //         new SequentialCommandGroup(
        //             followPath(
        //                 new Path(
        //                     new PathConstraints()
        //                         .setMaxVelocityMetersPerSec(AlignmentConstants.Tower.INTERMEDIARY_MAX_VELOCITY_METERS_PER_SEC)
        //                         .setEndTranslationToleranceMeters(AlignmentConstants.Tower.INTERMEDIARY_TRANSLATION_TOLERANCE_METERS)
        //                         .setEndRotationToleranceDeg(AlignmentConstants.Tower.INTERMEDIARY_ROTATION_TOLERANCE_DEG),
        //                     new Waypoint(AlignmentConstants.Tower.Left.INTERMEDIATE_WAYPOINT)),
        //                 false
        //             ),
        //             followPath(
        //                 new Path(
        //                     new PathConstraints().setMaxVelocityMetersPerSec(AlignmentConstants.Tower.APPROACH_MAX_VELOCITY_METERS_PER_SEC),
        //                     new Waypoint(AlignmentConstants.Tower.Left.FINAL_WAYPOINT)),
        //                 false
        //             )                
        //         ),
        //         new InstantCommand(() -> {}),
        //         () -> AlignmentConstants.Tower.isWithinBounds(robotState.getEstimatedPose(), AlignmentConstants.Tower.Left.BOUNDS)
        //     )
        // );

        // Test snap-to-angle bindings
        // xboxDriver.getAButton().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.BUMP)));
        // xboxDriver.getAButton().onFalse(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.DesiredState.HOME)));
    }

    private Command followPath(Path path, boolean shouldResetPose) {
        return 
            new InstantCommand(() -> {
                swerveDrive.setCurrentPath(path, shouldResetPose);
            })
            .andThen(setSwerveDriveState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)).
            andThen(new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE));
    }

    private Command setSwerveDriveState(SwerveDrive.DesiredSystemState state) {
        return new InstantCommand(() -> swerveDrive.setDesiredSystemState(state));
    }

    public void teleopInit() {
        // Ensure we're in teleop state
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
        superstructure.setDesiredState(Superstructure.DesiredState.HOME);
    }

    public void autonomousInit() {
        // Set up for autonomous
        superstructure.setDesiredState(DesiredState.HOME);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        superstructure.setDesiredState(Superstructure.DesiredState.DISABLED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
