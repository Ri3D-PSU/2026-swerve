package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax followerMotor;

    private final SparkMax feederMotor;
    private final SparkClosedLoopController shooterPID;

    private final GenericEntry IDLE_SPEED = Shuffleboard.getTab("Configuration")
            .add("IDLE SPEED", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();


    public Shooter() {
        shooterMotor = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pidf(0.1, 0, 0, 0.1); // TODO: tune
        shooterConfig.smartCurrentLimit(50);
        shooterConfig.voltageCompensation(12);
        shooterMotor.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        shooterPID = shooterMotor.getClosedLoopController();

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(shooterMotor, true); // TODO: check invert
        followerConfig.smartCurrentLimit(50);
        followerConfig.voltageCompensation(12);
        followerMotor.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);
        feederConfig.voltageCompensation(12);
        feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.setDefaultCommand(Commands.runOnce(() ->
                shooterPID.setReference(IDLE_SPEED.getDouble(0), SparkBase.ControlType.kMAXMotionVelocityControl))
        );
    }


    public Command setShooterSpeed(double speed) {
        return Commands.runOnce(
                () -> shooterPID.setReference(IDLE_SPEED.getDouble(0), SparkBase.ControlType.kMAXMotionVelocityControl),
                this
        );
    }

    private static final double FEEDER_FIRING_VOLTAGE = 8;


    public Command setFiring(boolean isFiring) {
        return Commands.runOnce(
                () -> feederMotor.setVoltage(isFiring ? FEEDER_FIRING_VOLTAGE : 0), this
        );
    }
}