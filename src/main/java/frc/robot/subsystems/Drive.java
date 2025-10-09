package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private Module frontLeftModule;
    private Module frontRightModule;
    private Module backLeftModule;
    private Module backRightModule;
    private static final double TRACK_WIDTH_X = Units.inchesToMeters(31.0);
    private static final double TRACK_WIDTH_Y = Units.inchesToMeters(27.0);
    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(17.2);
    private static final double MAX_ANGULAR_SPEED = Math.PI;
    private Translation2d[] modulePositions;
    private SwerveDriveKinematics kinematics;
    
    private AHRS gyro;

    
    public Drive() {
        frontLeftModule = new Module(0);
        frontRightModule = new Module(1);
        backLeftModule = new Module(2);
        backRightModule = new Module(3);

        modulePositions = new Translation2d[] {
            new Translation2d(-TRACK_WIDTH_X/2, TRACK_WIDTH_Y/2),
            new Translation2d(TRACK_WIDTH_X/2, TRACK_WIDTH_Y/2),
            new Translation2d(-TRACK_WIDTH_X/2, -TRACK_WIDTH_Y/2),
            new Translation2d(-TRACK_WIDTH_X/2, -TRACK_WIDTH_Y/2)
        };

        kinematics = new SwerveDriveKinematics(modulePositions);

        gyro = new AHRS(NavXComType.kMXP_SPI);
        

    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);
    }

    public void drive(double x, double y, double theta, boolean isFieldOriented) {
        double xSpeed = MAX_LINEAR_SPEED*x;
        double ySpeed = MAX_LINEAR_SPEED*y;
        double thetaSpeed = MAX_ANGULAR_SPEED*theta;

        if(isFieldOriented) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, gyro.getRotation2d()));
        }
        else {
            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));
        }
        
    }



}
