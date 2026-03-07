package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autos.AutoShootingController;
import frc.robot.commands.autos.AutoShootingZoneManager;
import frc.robot.commands.autos.tower.ScoreL1;
import frc.robot.constants.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

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

    // private final Shooter shooter = Shooter.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    // private final Superstructure superstructure = Superstructure.getInstance();
    private final Vision vision = Vision.getInstance();

    // Path for follow path state
    private Path currentPath = null;
    private boolean shouldResetPose = false;

    private RobotContainer() {
        registerEventTriggers();

        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        // Configure teleop input suppliers for SwerveDrive FSM
        // Using normalized inputs (-1 to 1) with deadband applied
        swerveDrive.setTeleopInputSuppliers(
            () ->
                -MathUtil.applyDeadband(
                    xboxDriver.getLeftY(),
                    Constants.OperatorConstants.LEFT_Y_DEADBAND
                ),
            () ->
                -MathUtil.applyDeadband(
                    xboxDriver.getLeftX(),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
                ),
            () ->
                -MathUtil.applyDeadband(
                    xboxDriver.getRightX(),
                    Constants.OperatorConstants.RIGHT_X_DEADBAND
                )
        );

        // Set up path supplier for SwerveDrive
        swerveDrive.setPathSupplier(() -> currentPath, () -> shouldResetPose);

        // Set default state to TELEOP
        swerveDrive.setDesiredSystemState(
            SwerveDrive.DesiredSystemState.TELEOP
        );

        // Set default superstructure state to HOME
        // superstructure.setDesiredState(Superstructure.DesiredState.HOME);

        configureBindings();
    }

    private void registerEventTriggers() {
        // register shooting trigger (up to 5 per path but change if necessary lol)
        for (int i = 0; i < 5; i++) {
            final int zoneId = i;
            FollowPath.registerEventTrigger(
                "shooting_zone_" + i + "_start",
                () -> {
                    AutoShootingZoneManager.getInstance().enterShootingZone(
                        zoneId
                    );
                }
            );
            FollowPath.registerEventTrigger(
                "shooting_zone_" + i + "_end",
                () -> {
                    AutoShootingZoneManager.getInstance().exitShootingZone(
                        zoneId
                    );
                }
            );
        }

        FollowPath.registerEventTrigger(
            "test1",
            new InstantCommand(() -> {
                Logger.recordOutput("In a command", true);
            })
        );
        FollowPath.registerEventTrigger("test2", () -> {
            System.out.println("As a runnable");
        });

        FollowPath.registerEventTrigger("start_intake", () -> {
            Intake.getInstance().setDesiredState(
                Intake.IntakeDesiredState.INTAKING
            );
        });

        FollowPath.registerEventTrigger("start_shooter", () -> {
            Superstructure.getInstance().setDesiredState(
                Superstructure.DesiredState.READY_FOR_SHOT
            );
        });

        FollowPath.registerEventTrigger("stop_shooter", () -> {
            Superstructure.getInstance().setDesiredState(
                Superstructure.DesiredState.HOME
            );
        });
    }

    private void configureBindings() {
        // Path toClimb = new Path(new Waypoint(1,3, new Rotation2d(0)));
        // xboxDriver.getXButton().onTrue(
        // new InstantCommand(() -> currentPath = toClimb).andThen(
        // new InstantCommand(() ->
        // swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH)).andThen(
        // new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() ==
        // SwerveDrive.CurrentSystemState.IDLE)).andThen(
        // new InstantCommand(() ->
        // swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.TELEOP))
        // )
        // )
        // );

        xboxDriver
            .getXButton()
            .onFalse(
                new InstantCommand(() ->
                    robotState.resetPose(new Pose2d(0, 0, new Rotation2d(0)))
                )
            );
    }

    public void teleopInit() {
        shouldResetPose = false;

        // Ensure we're in teleop state
        swerveDrive.setDesiredSystemState(
            SwerveDrive.DesiredSystemState.TELEOP
        );
        // superstructure.setDesiredState(Superstructure.DesiredState.HOME);
    }

    public void autonomousInit() {
        // Set up for autonomous
        // superstructure.setDesiredState(DesiredState.HOME);
        swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE);
    }

    public void disabledInit() {
        // superstructure.setDesiredState(Superstructure.DesiredState.DISABLED);
        swerveDrive.setDesiredSystemState(
            SwerveDrive.DesiredSystemState.DISABLED
        );
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("TRIGGER1", false);
        Logger.recordOutput("TRIGGER2", false);

        // Reset shooting zone state at start of auto
        AutoShootingZoneManager.getInstance().reset();

        currentPath = new Path("bottom_start_to_output_shoot");
        shouldResetPose = true;

        Command pathFollowingCommand = new InstantCommand(() ->
            swerveDrive.setDesiredSystemState(
                SwerveDrive.DesiredSystemState.PREPARE_FOR_AUTO
            )
        )
            .andThen(
                new WaitUntilCommand(
                    () ->
                        swerveDrive.getCurrentSystemState() ==
                        SwerveDrive.CurrentSystemState.READY_FOR_AUTO
                )
            )
            .andThen(
                new InstantCommand(() ->
                    swerveDrive.setDesiredSystemState(
                        SwerveDrive.DesiredSystemState.FOLLOW_PATH
                    )
                )
            );

        return new ParallelCommandGroup(
            new AutoShootingController(),
            pathFollowingCommand
        );
    }
}
