package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        field.setRobotPose(poseEstimator.getEstimatedPosition());
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
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, gyro.getRotation2d().times(-1)));
        } else {
            drive(new ChassisSpeeds(x, y, theta));
        }

    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }


}
