package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import static frc.robot.Constants.LIMELIGHT_NAME;

public class Drive extends SubsystemBase {
    private final Module frontLeftModule;
    private final Module frontRightModule;
    private final Module backLeftModule;
    private final Module backRightModule;
    private static final double TRACK_WIDTH_X = Units.inchesToMeters(31.0);
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
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d());
    }


    @Override
    public void periodic() {
        SwerveModuleState[] states = new SwerveModuleState[]
                {
                        frontLeftModule.getState(),
                        frontRightModule.getState(),
                        backLeftModule.getState(),
                        backRightModule.getState()
                };
        moduleStatePublisher.set(states);

        poseEstimator.update(gyro.getRotation2d(), getModulePositions());


        // Needed for doing megatag2
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, getGyroRotation().getDegrees(), 0, 0, 0, 0, 0);
        var megaTag2VisionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        var megaTag1VisionPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        Logger.recordOutput("MegaTag2Pose", megaTag2VisionPose.pose);
        Logger.recordOutput("MegaTag1Pose", megaTag1VisionPose.pose);

        if (megaTag2VisionPose.timestampSeconds != lastVisionTimestamp && megaTag2VisionPose.tagCount > 0) {
            lastVisionTimestamp = megaTag2VisionPose.timestampSeconds;
            Logger.recordOutput("LimelightPose", megaTag2VisionPose.pose);
            var visionStd = calculateVisionStd(megaTag2VisionPose);

            var newVisionPose = megaTag2VisionPose.pose;
            if (visionStd.get(0, 3) < 10.0) {
                // let's use the megatag1 rotation to update our gyro angle
                newVisionPose = new Pose2d(
                        megaTag2VisionPose.pose.getTranslation(),
                        megaTag1VisionPose.pose.getRotation()
                );
            }
            poseEstimator.addVisionMeasurement(newVisionPose, megaTag2VisionPose.timestampSeconds, visionStd);
        }
        Logger.recordOutput("Last Vision Update", lastVisionTimestamp);

        field.setRobotPose(poseEstimator.getEstimatedPosition()); // Logs the position for advantagekit
    }


    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);

        desiredStatePublisher.set(states);

    }

    public void drive(double x, double y, double theta, boolean isFieldOriented) {
        if (isFieldOriented) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getGyroRotation()));
        } else {
            drive(new ChassisSpeeds(x, y, theta));
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }


    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d().times(-1);
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

        if (closestTag.distToCamera < 2.0 && rotationRate < Math.PI * 0.25) {
            rotationStd = Math.max(3.0 * Math.log(closestTag.distToCamera), 1.0);
        }

        var positionStd = 0.1 * Math.log(closestTag.distToCamera) * (1 + rotationRate * 2);

        return new Matrix<>(Nat.N3(), Nat.N1(), new double[]{positionStd, positionStd, rotationStd});
    }

}
