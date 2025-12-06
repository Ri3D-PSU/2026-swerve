package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Module {

    private SparkMax drivingSparkMax;
    private SparkMax turningSparkMax;
    private final AbsoluteEncoder turningEncoder;
    private final RelativeEncoder drivingEncoder;
    private final SparkClosedLoopController drivingPID;
    private final SparkClosedLoopController turningPID;
    private SwerveModuleState desiredState;
    private static final double DRIVE_GEAR_RATIO = 5.9;


    public Module(int corner) {
        switch (corner) {
            case 0 -> {
                drivingSparkMax = new SparkMax(19, MotorType.kBrushless);
                turningSparkMax = new SparkMax(18, MotorType.kBrushless);
            }
            case 1 -> {
                drivingSparkMax = new SparkMax(29, MotorType.kBrushless);
                turningSparkMax = new SparkMax(28, MotorType.kBrushless);
            }
            case 2 -> {
                drivingSparkMax = new SparkMax(11, MotorType.kBrushless);
                turningSparkMax = new SparkMax(12, MotorType.kBrushless);
            }
            case 3 -> {
                drivingSparkMax = new SparkMax(21, MotorType.kBrushless);
                turningSparkMax = new SparkMax(22, MotorType.kBrushless);
            }
        }

        drivingEncoder = drivingSparkMax.getEncoder();
        turningEncoder = turningSparkMax.getAbsoluteEncoder();
        drivingPID = drivingSparkMax.getClosedLoopController();
        turningPID = turningSparkMax.getClosedLoopController();

        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        drivingConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(40)
                .voltageCompensation(12);

        drivingConfig.closedLoop
                .pidf(0, 0, 0, 0.187)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1, 1);

        drivingConfig.encoder
                .positionConversionFactor(1. / DRIVE_GEAR_RATIO * Units.inchesToMeters(4 * Math.PI))
                .velocityConversionFactor(1. / DRIVE_GEAR_RATIO * Units.inchesToMeters(4 * Math.PI) / 60.);


        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(30)
                .voltageCompensation(12);

        turningConfig.closedLoop
                .pidf(0.8, 0, 0, 0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);

        turningConfig.absoluteEncoder
                .positionConversionFactor(Units.rotationsToRadians(1))
                .inverted(true);


        System.out.println(drivingSparkMax.getDeviceId() + " driving " + drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        System.out.println(turningSparkMax.getDeviceId() + " turning " + turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        desiredState = new SwerveModuleState(0, new Rotation2d(turningEncoder.getPosition()));

    }


    public void setDesiredState(SwerveModuleState targetState, boolean shouldTurn) {
        desiredState = targetState;
        desiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingPID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        if (shouldTurn) {
            turningPID.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }


    public void setIdleMode(IdleMode idleMode) {

    }

}
