package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Utils;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class Drive extends SubsystemBase {
    private final Module frontLeftModule;
    private final Module frontRightModule;
    private final Module backLeftModule;
    private final Module backRightModule;
    private static final double TRACK_WIDTH_X = Units.inchesToMeters(27.0);
    private static final double TRACK_WIDTH_Y = Units.inchesToMeters(27.0);
    private final SwerveDriveKinematics kinematics;
    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("currentStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("desiredStates", SwerveModuleState.struct).publish();
    private final AHRS gyro;


    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private double lastVisionTimestamp = -1;


    public Drive() {
        frontLeftModule = new Module(0);
        frontRightModule = new Module(1);
        backLeftModule = new Module(2);
        backRightModule = new Module(3);

        gyro = new AHRS(NavXComType.kMXP_SPI);

        Translation2d[] moduleLocations = new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
                new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
                new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
                new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
        };

        kinematics = new SwerveDriveKinematics(moduleLocations);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            throw new RuntimeException(e);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );

    }


    @Override
    public void periodic() {
        SwerveModuleState[] states = getSwerveModuleStates();
        moduleStatePublisher.set(states);

        Logger.recordOutput("Gyro Rotation", gyro.getRotation2d());
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());


        // Needed for doing megatag2
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, getGyroRotation().getDegrees(), 0, 0, 0, 0, 0);
        var megaTag2VisionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        var megaTag1VisionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (megaTag2VisionPose != null && megaTag1VisionPose != null && megaTag2VisionPose.timestampSeconds != lastVisionTimestamp && megaTag2VisionPose.tagCount > 0) {
            lastVisionTimestamp = megaTag2VisionPose.timestampSeconds;
            Logger.recordOutput("LimelightPose", megaTag2VisionPose.pose);
            var visionStd = calculateVisionStd(megaTag2VisionPose);

            var newVisionPose = megaTag2VisionPose.pose;
            if (visionStd.get(2, 0) < 10.0) {
                // let's use the megatag1 rotation to update our gyro angle
                newVisionPose = new Pose2d(
                        megaTag2VisionPose.pose.getTranslation(),
                        megaTag1VisionPose.pose.getRotation()
                );
            }
            poseEstimator.addVisionMeasurement(newVisionPose, megaTag2VisionPose.timestampSeconds, visionStd);

            Logger.recordOutput("MegaTag2Pose", megaTag2VisionPose.pose);
            Logger.recordOutput("MegaTag1Pose", megaTag1VisionPose.pose);

        }
        Logger.recordOutput("Last Vision Update", lastVisionTimestamp);
        Logger.recordOutput("Fused Pose", poseEstimator.getEstimatedPosition());

        field.setRobotPose(poseEstimator.getEstimatedPosition()); // Logs the position for advantagekit
    }


    /**
     * @param speeds robot relative speeds
     */
    public void drive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, TICK_TIME);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        boolean shouldTurn = Math.abs(speeds.vyMetersPerSecond) + Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.omegaRadiansPerSecond) > 0.0;
        frontLeftModule.setDesiredState(states[0], shouldTurn);
        frontRightModule.setDesiredState(states[1], shouldTurn);
        backLeftModule.setDesiredState(states[2], shouldTurn);
        backRightModule.setDesiredState(states[3], shouldTurn);

        desiredStatePublisher.set(states);

    }

    public void drive(double x, double y, double theta, boolean isFieldOriented) {
        if (isFieldOriented) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getGyroRotation()));
        } else {
            drive(new ChassisSpeeds(x, y, theta));
        }
    }

    public void rotationPidDrive(double x, double y, double angle, double angularVelocity, double angularAcceleration) {
        double kV = 0.0;
        double kA = 0.0;
        var thetaSpeed = positionRotationPid.calculate(getPose().getRotation().getRadians(), angle) + angularVelocity * kV + angularAcceleration * kA ;
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, thetaSpeed, getGyroRotation()));
    }


    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        };
    }


    public Rotation2d getGyroRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose2d) {
        poseEstimator.resetPose(pose2d);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getGyroRotation());
    }

    PIDController positionTranslationPid = new PIDController(5.0, 0.0, 0.0);

    PIDController positionRotationPid = new PIDController(5.0, 0.0, 0.1);



    {
        positionRotationPid.enableContinuousInput(0, Units.degreesToRadians(360.0));
        SmartDashboard.putData(positionRotationPid);
        SmartDashboard.putData(positionTranslationPid);
    }

    public Command pidToPosition(Pose2d goal) {
        return Commands.run(
                () -> drive(
                        positionTranslationPid.calculate(getPose().getX(), goal.getX()),
                        positionTranslationPid.calculate(getPose().getY(), goal.getY()),
                        positionRotationPid.calculate(getPose().getRotation().getRadians(), goal.getRotation().getRadians()),
                        true
                ),
                this
        );
    }


    /**
     * Precondition: visionPose.rawFiducials.length > 0
     *
     * @param visionPose
     * @return Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
     */
    private Matrix<N3, N1> calculateVisionStd(LimelightHelpers.PoseEstimate visionPose) {

        var closestTag = visionPose.rawFiducials[0];

        for (LimelightHelpers.RawFiducial rawFiducial : visionPose.rawFiducials) {
            if (closestTag.distToCamera < rawFiducial.distToCamera) {
                closestTag = rawFiducial;
            }
        }

        // std calculation are just guesses
        var rotationStd = 99999.0;

        var rotationRate = Math.toRadians(gyro.getRate());

        if (closestTag.distToCamera < 1 && rotationRate < Math.PI * 0.25) {
            rotationStd = Math.max(15.0 * Math.log(closestTag.distToCamera), 1.0);
        }

        var positionStd = 0.3 * Math.log(closestTag.distToCamera) * (1 + rotationRate * 2);

        return new Matrix<>(Nat.N3(), Nat.N1(), new double[]{positionStd, positionStd, rotationStd});
    }

    public void setBrakeMode(SparkBaseConfig.IdleMode idleMode) {
        frontLeftModule.setIdleMode(idleMode);
        frontRightModule.setIdleMode(idleMode);
        backLeftModule.setIdleMode(idleMode);
        backRightModule.setIdleMode(idleMode);
    }

    public void zeroPose() {
        poseEstimator.resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public boolean shouldBumpAdjust() {
        var futurePos = getPose().getTranslation().plus(new Translation2d(getFieldRelativeSpeeds().vxMetersPerSecond*0.5, getFieldRelativeSpeeds().vyMetersPerSecond*0.5));
        return (Utils.isPointInBox(futurePos, new Translation2d(), new Translation2d()));
    }

    public double closestBumpAngle() {
        double shiftedAngle = getPose().getRotation().getRadians() - Math.PI/4;
        double snappedShifted = Math.round(shiftedAngle / (Math.PI / 2.0)) * (Math.PI / 2.0);
        return  snappedShifted + Math.PI/4;
    }
}
