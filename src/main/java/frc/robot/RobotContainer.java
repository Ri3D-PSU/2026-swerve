package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.concurrent.Future;

import static frc.robot.Constants.*;
import static frc.robot.Utils.getSpeed2;

public class RobotContainer {

    private final Drive drive = new Drive();
    private final Shooter shooter = new Shooter();
    private final Climb climb = new Climb(drive);
    private final Intake intake = new Intake();

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final Command shootCommand = new ShootWhileMoving(drive, shooter, m_driverController, intake);

    private Future<PathPlannerPath> onTheFlyPath = null;

    public Command getDriveToGoal(Pose2d finalPathPoint, Rotation2d targetRotation, PathConstraints constraints) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                onTheFlyPath = AsyncPathGenerator.generatePathAsync(finalPathPoint, targetRotation, drive,
                    constraints);
            }),
            Commands.waitUntil(() -> onTheFlyPath.isDone()),
            Commands.race(
                Commands.defer(
                    () -> {
                        try {
                            return AutoBuilder.followPath(onTheFlyPath.get());
                        } catch (Exception e) {
                            throw new RuntimeException(e);
                        }
                    },
                    Set.of(drive)),
                Commands.waitUntil(
                    () -> drive.getPose().getTranslation().minus(finalPathPoint.getTranslation())
                        .getNorm() < Constants.PATH_FINISH_CLOSE_DISTANCE_M &&
                        MathUtil.angleModulus(drive.getPose().getRotation().minus(targetRotation)
                            .getRadians()) < ANGLE_CLOSE_RAD
                        &&
                        getSpeed2(drive.getRobotRelativeSpeeds()) < 0.1)),
            Commands.race(
                drive.pidToPosition(new Pose2d(finalPathPoint.getTranslation(), targetRotation)),
                Commands.waitUntil(
                    () -> drive.getPose().getTranslation().minus(finalPathPoint.getTranslation())
                        .getNorm() < Constants.PATH_FINISH_CLOSE_DISTANCE_M_PID)));
    }

    public RobotContainer() {
        NamedCommands.registerCommand("shoot command", Commands.repeatingSequence(new ShootWhileMoving(drive, shooter, m_driverController, intake)).withTimeout(6));
        NamedCommands.registerCommand("toggle intake", intake.toggleIntake());
        NamedCommands.registerCommand("intake", Commands.repeatingSequence(intake.intake()).withTimeout(3));


        // Warm up path planner
        System.out.println("Warming up path planner");
        for (int i = 0; i < 10; i++) {
            var finalPathPoint = new Pose2d(
                new Translation2d(Math.random() * 10 - 5, Math.random() * 10 - 5),
                Rotation2d.fromDegrees(Math.random() * 360));

            Rotation2d targetRotation = Rotation2d.fromDegrees(Math.random() * 360);
            var time = Timer.getFPGATimestamp();
            var path = AsyncPathGenerator.generatePathAsync(finalPathPoint, targetRotation, drive, normConstraints);
            try {
                var points = path.get().getAllPathPoints();
                var duration = Timer.getFPGATimestamp() - time;
                System.out.println(
                    "Generated path " + (i + 1) + "/10. " + points.size() + " points in " + duration + " seconds");
            } catch (Exception e) {
                System.out.println("Path failed to generate" + e);
            }
        }


        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(
            new RunCommand(() -> {
                var thetaInput = Math.abs(m_driverController.getRightX()) * m_driverController.getRightX()
                    * MAX_ANGULAR_SPEED * -1;
                var controls = getControls(m_driverController);
                if (drive.shouldBumpAdjust()) {
                    drive.rotationPidDrive(controls.getX(), controls.getY(), drive.closestBumpAngle(), 0.0, 0.0);
                } else {
                    drive.drive(controls.getX(), controls.getY(), thetaInput, true);
                }

            }, drive));

        // m_driverController.b().whileTrue(
        // getDriveToGoal(new Pose2d(new Translation2d(14.8, 4.09),
        // Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0))
        // );
        var restHeadingCommand = Commands.runOnce(drive::zeroPose, drive);
        restHeadingCommand.runsWhenDisabled();
        m_driverController.start().onTrue(restHeadingCommand);

        m_driverController.leftTrigger().whileTrue(intake.intake());
        m_driverController.leftBumper().whileTrue(intake.outtake());
        m_driverController.rightBumper().onTrue(intake.toggleIntake());

        m_driverController.rightTrigger().whileTrue(Commands.parallel(shootCommand));

        m_driverController.a().whileTrue(Commands.run(
            () -> {
                boolean isAtSpeed = shooter.isAtSpeed(1700);
                shooter.setShooterSpeed(1700, false);
                shooter.setFiring(isAtSpeed);
            }, shooter));

        m_driverController.y().whileTrue(Commands.runEnd(
            () -> {
                shooter.setShooterSpeed(-1700, false);
                shooter.setOuttake();
            }, () -> {
                shooter.setShooterSpeed(0, false);
                shooter.setFiring(false);
            }, shooter));


        m_driverController.x().whileTrue(Commands.run(
            () -> {
                shooter.setShooterSpeed(1700, false);
            }, shooter));

        m_driverController.b().whileTrue(Commands.run(() -> shooter.setFiring(true)));

        // m_driverController.a().whileTrue(Commands.sequence(
        // Commands.race(
        // getDriveToGoal(new Pose2d(6, 7, Rotation2d.fromDegrees(90)),
        // Rotation2d.fromDegrees(0), normConstraints),
        // Commands.waitUntil(() ->
        // LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME) == 15)
        // ),
        // Commands.waitSeconds(0.5),
        // getDriveToGoal(new Pose2d(6, 7, Rotation2d.fromDegrees(90)),
        // Rotation2d.fromDegrees(0), normConstraints),
        // climb.extend(false)
        // ));

        // m_driverController.x().whileTrue(Commands.sequence(
        // Commands.parallel(
        // climb.setVoltageWithFeedforward(3, drive).until(() ->
        // drive.getGyroPitch().getDegrees() >= 180),
        // Commands.run(() -> drive.drive(0.2, 0, 0, true)).until(() ->
        // drive.getGyroPitch().getDegrees() >= 30)
        // ),
        // climb.fixPIDPositionReference(drive.getGyroPitch().getRadians())
        // ));

        m_driverController.povLeft().onTrue(climb.extend(false));
        m_driverController.povUp().whileTrue(climb.setVoltageWithFeedforward(12, drive, true)
            .alongWith(climb.extend(true)));
        m_driverController.povDown().onTrue(Commands.run(() -> {
            shooter.setFiring(false);
            shooter.setShooterSpeed(0, false);
        }, shooter));
        m_driverController.povDown().whileTrue(climb.setVoltageWithFeedforward(-7, drive, false));

    }

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

    }

    public static Translation2d getControls(CommandXboxController m_driverController) {

        // var yInput = Math.abs(m_driverController.getLeftY()) *
        // m_driverController.getLeftY() * MAX_LINEAR_SPEED_TELEOP;
        // var xInput = -Math.abs(m_driverController.getLeftX()) *
        // m_driverController.getLeftX() * MAX_LINEAR_SPEED_TELEOP;
        var isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        double xInput;
        double yInput;
        if (!isRed) {
            xInput = -Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY()
                * MAX_LINEAR_SPEED_TELEOP;
            yInput = -Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX()
                * MAX_LINEAR_SPEED_TELEOP;
        } else {
            xInput = Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY()
                * MAX_LINEAR_SPEED_TELEOP;
            yInput = Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX()
                * MAX_LINEAR_SPEED_TELEOP;
        }

        return new Translation2d(xInput, yInput);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        var isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        double xInput;
        double theta;
        if (isRed) {
            xInput = 0.5;
            theta = 180;
        } else {
            xInput = -0.5;
            theta = 0;
        }

        return AutoBuilder.buildAuto("New Auto");
//
//        return Commands.sequence(
//            Commands.runOnce(() -> drive.resetPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY(), Rotation2d.fromDegrees(theta)))),
//            Commands.repeatingSequence(new ShootWhileMoving(drive, shooter, m_driverController, intake)).withTimeout(6)
//            //AutoBuilder.followPath(PathPlannerPath.fromPathFile("p1"))
//        );
    }

    public void disabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kCoast))
                .beforeStarting(Commands.waitSeconds(3.0))
                .ignoringDisable(true));
    }

    public void enabledInit() {
        CommandScheduler.getInstance().schedule(
            Commands.runOnce(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kBrake)));
    }
}
