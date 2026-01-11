package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive;

import java.util.Set;
import java.util.concurrent.Future;

import static frc.robot.Constants.*;


public class RobotContainer {

    private final Drive drive = new Drive();


    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private Future<PathPlannerPath> onTheFlyPath = null;

    public Command getDriveToGoal(Pose2d finalPathPoint, Rotation2d targetRotation) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    onTheFlyPath = AsyncPathGenerator.generatePathAsync(finalPathPoint, targetRotation, drive);
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
                                Set.of(drive)
                        ),
                        Commands.waitUntil(
                                () -> drive.getPose().getTranslation().minus(finalPathPoint.getTranslation()).getNorm()
                                        < Constants.PATH_FINISH_CLOSE_DISTANCE_M
                        )
                ),
                Commands.race(
                        drive.pidToPosition(new Pose2d(finalPathPoint.getTranslation(), targetRotation)),
                        Commands.waitUntil(
                                () -> drive.getPose().getTranslation().minus(finalPathPoint.getTranslation()).getNorm()
                                        < Constants.PATH_FINISH_CLOSE_DISTANCE_M_PID
                        )
                )
        );
    }

    public RobotContainer() {
        // Warm up path planner
        System.out.println("Warming up path planner");
        for (int i = 0; i < 10; i++) {
            var finalPathPoint = new Pose2d(
                    new Translation2d(Math.random() * 10 - 5, Math.random() * 10 - 5),
                    Rotation2d.fromDegrees(Math.random() * 360)
            );

            Rotation2d targetRotation = Rotation2d.fromDegrees(Math.random() * 360);
            var time = Timer.getFPGATimestamp();
            var path = AsyncPathGenerator.generatePathAsync(finalPathPoint, targetRotation, drive);
            try {
                var points = path.get().getAllPathPoints();
                var duration =  Timer.getFPGATimestamp() - time;
                System.out.println("Generated path " + (i + 1) + "/10. " + points.size() + " points in " + duration + " seconds");
            } catch (Exception e) {
                System.out.println("Path failed to generate" + e);
            }
        }

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(
                new RunCommand(() -> drive.drive(
                        -Math.abs(m_driverController.getLeftY()) * m_driverController.getLeftY() * MAX_LINEAR_SPEED_TELEOP,
                        -Math.abs(m_driverController.getLeftX()) * m_driverController.getLeftX() * MAX_LINEAR_SPEED_TELEOP,
                        Math.abs(m_driverController.getRightX()) * m_driverController.getRightX() * MAX_ANGULAR_SPEED * -1,
                        true), drive));
        m_driverController.b().whileTrue(
                getDriveToGoal(new Pose2d(new Translation2d(14.8, 4.09), Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0))
        );

        m_driverController.start().onTrue(Commands.runOnce(drive::zeroPose, drive));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }


    public void disabledInit() {
        CommandScheduler.getInstance().schedule(
                Commands.run(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kCoast))
                        .beforeStarting(Commands.waitSeconds(3.0))
                        .ignoringDisable(true)
        );
    }

    public void enabledInit() {
        CommandScheduler.getInstance().schedule(
                Commands.run(() -> drive.setBrakeMode(SparkBaseConfig.IdleMode.kBrake))
        );
    }
}
