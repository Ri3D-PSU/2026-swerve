package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.SHOOT_ANGLE_RANGE_RAD;
import static frc.robot.RobotContainer.getControls;
import static frc.robot.RobotContainer.isRed;

public class ShootWhileMoving extends Command {
    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final CommandXboxController controller;

    private static final Translation2d TARGET_POS_RED = new Translation2d(
            Units.inchesToMeters(458.56),
            Units.inchesToMeters(158.84));

    private static final Translation2d TARGET_POS_BLUE = new Translation2d(
            Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.84));
    private static final double FEEDER_HAS_BALL_CURRENT_THRESHOLD = 30; // TODO tune
    private static final double SHOOT_BOOST_TIME_S = 0.4;
    private double boostTillTime = 0;

    public ShootWhileMoving(Drive drive, Shooter shooter, CommandXboxController controller, Intake intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.controller = controller;
        this.intake = intake;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        boostTillTime = 0;
    }

    @Override
    public void execute() {

        // Prediction Math
        Translation2d robotVelocity = new Translation2d(drive.getFieldRelativeSpeeds().vxMetersPerSecond,
                drive.getFieldRelativeSpeeds().vyMetersPerSecond);
        Translation2d robotPositionT0 = drive.getPose().getTranslation();

        var linearVelocity = robotVelocity.getNorm();
        var linnearAccel = drive.getGyroAcceleration();
        double timeTillStop;
        if (linnearAccel < 0.05) {
            timeTillStop = 0.3;
        } else {
            timeTillStop = Math.min(0.3, linearVelocity / linnearAccel);
        }

        Translation2d targetPos = isRed() ? TARGET_POS_RED : TARGET_POS_BLUE;

        Translation2d robotShootPosition = robotPositionT0.plus(robotVelocity.times(timeTillStop));
        Rotation2d targetRotation = targetPos.minus(robotShootPosition).getAngle();
        double wantedShooterVelocity = getWantedShooterVelocity(targetPos);

        // Drive Control (Strafing + Aiming)
        // Note: We use the controller inputs for X/Y translation, but override rotation
        var controls = getControls(controller);
        drive.rotationPidDrive(controls.getX(), controls.getY(), targetRotation.getRadians(), 0, 0);
        var robotPose = drive.getPose();
        var distance = robotPose.getTranslation().getDistance(targetPos);

        // Shooter Control
        double angleError = MathUtil.angleModulus(drive.getPose().getRotation().minus(targetRotation).getRadians());
        boolean isAligned = Math.abs(angleError) < SHOOT_ANGLE_RANGE_RAD;
        boolean isAtSpeed = shooter.isAtSpeed(wantedShooterVelocity);

        if (shooter.getFeederCurrent() > FEEDER_HAS_BALL_CURRENT_THRESHOLD) {
            boostTillTime = Timer.getFPGATimestamp() + SHOOT_BOOST_TIME_S;
        }

        shooter.setShooterSpeed(wantedShooterVelocity, boostTillTime > Timer.getFPGATimestamp());

        boolean shouldFire = isAligned && isAtSpeed && distance > 2.5 && linearVelocity < 0.1;
        shooter.setFiring(shouldFire);

        Logger.recordOutput("Shooter/Wanted Velocity", wantedShooterVelocity);
        Logger.recordOutput("Shooter/Angle Error", angleError);
        Logger.recordOutput("Shooter/Is Aligned", isAligned);
        Logger.recordOutput("Shooter/Is At Speed", isAtSpeed);
        Logger.recordOutput("Shooter/Ready To Fire", shouldFire);
        Logger.recordOutput("Shooter/Distance", distance);
        Logger.recordOutput("Shooter/timeTillStop", timeTillStop);
        Logger.recordOutput("Shooter/robotShootPosition", robotShootPosition);
        Logger.recordOutput("Linear Velocity", linearVelocity);
        Logger.recordOutput("Linear Acceleration", drive.getGyroAcceleration());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFiring(false);
    }

    private double getWantedShooterVelocity(Translation2d targetPosition) {
        var robotPose = drive.getPose();
        var distance = robotPose.getTranslation().getDistance(targetPosition);
        return shooter.getWantedVelocity(distance);
    }
}
