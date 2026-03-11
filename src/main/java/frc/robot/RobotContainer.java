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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.BallTargeting;
import frc.robot.subsystems.vision.Vision;

//: TODO: make sure all lisences in tuner are activated
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
    // private final Superstructure superstructure = Superstructure.getInstance();
    private final Vision vision = Vision.getInstance();
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
                () -> -MathUtil.applyDeadband(xboxDriver.getRightX(), Constants.OperatorConstants.RIGHT_X_DEADBAND));

        // Set default state to TELEOP
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);

        // Set default superstructure state to HOME
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        // superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED);

        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP);

        configureBindings();
    }

    private void registerEventTriggers() {
        // FollowPath.registerEventTrigger("test1", new InstantCommand(() -> {
        // Logger.recordOutput("In a command", true);
        // }));
        // FollowPath.registerEventTrigger("test2", () -> {
        // System.out.println("As a runnable");
        // });
    }

    private void configureBindings() {
        xboxDriver.getXButton().onTrue(
                new InstantCommand(() -> robotState
                        .resetPose(new Pose2d(robotState.getEstimatedPose().getTranslation(), new Rotation2d(0)))));
        // xboxDriver.getAButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME))
        // );
        // xboxDriver.getBButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING))
        // );

        // xboxOperator.getYButton().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.HUB))
        // );
        // xboxOperator.getUpDpad().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_TOP))
        // );
        // xboxOperator.getRightDpad().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_CENTER))
        // );
        // xboxOperator.getDownDpad().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_ALLIANCE_BOTTOM))
        // );
        // xboxOperator.getLeftBumper().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_TOP))
        // );
        // xboxOperator.getLeftDpad().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_CENTER))
        // );
        // xboxOperator.getRightBumper().onTrue(
        // new InstantCommand(() ->
        // superstructure.setDesiredTargetState(Superstructure.TargetState.PASS_NEUTRAL_BOTTOM))
        // );

        // Split-state pattern example:
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.TRACKING);
        // superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.INTAKING);

        // xboxDriver.getAButton().onTrue(
        // new InstantCommand(() ->
        // Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.INTAKING))
        // );
        // xboxDriver.getBButton().onTrue(
        // new InstantCommand(() ->
        // Intake.getInstance().setSetpoint(Intake.IntakeSetpoint.STOWED))
        // );

        // Test snap-to-angle bindings with split superstructure states:
        // xboxDriver.getAButton().onTrue(new InstantCommand(() ->
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.BUMP)));
        // xboxDriver.getAButton().onFalse(new InstantCommand(() ->
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)));

        // Vision-assisted ball alignment: enable assist while intake trigger is held
        xboxDriver.getRightTriggerButton(0.5)
                .onTrue(new InstantCommand(() -> BallTargeting.getInstance().setAssistEnabled(true)))
                .onFalse(new InstantCommand(() -> BallTargeting.getInstance().setAssistEnabled(false)));
    }

    private Command followPath(Path path, boolean shouldResetPose) {
        return new InstantCommand(() -> {
            swerveDrive.setCurrentPath(path, shouldResetPose);
        })
                .andThen(setSwerveDriveState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)).andThen(new WaitUntilCommand(
                        () -> swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE));
    }

    private Command setSwerveDriveState(SwerveDrive.DesiredSystemState state) {
        return new InstantCommand(() -> swerveDrive.setDesiredSystemState(state));
    }

    public void teleopInit() {
        // Ensure we're in teleop state
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP);
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
    }

    public void autonomousInit() {
        // Set up for autonomous
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        // superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.DISABLED);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.DISABLED);
    }

    public Command getAutonomousCommand() {
        return followPath(new Path("1"), true).andThen(followPath(new Path("2"), false));
        // return null;
    }
}
