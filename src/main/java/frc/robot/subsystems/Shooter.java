package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static frc.robot.Constants.SHOOTER_VELOCITY_RANGE;

public class Shooter extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax followerMotor;

    private final SparkMax feederMotor;
    private final SparkClosedLoopController shooterPID;

    private final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
    private double[] lastMapData = new double[0];

    private final GenericEntry SPEED_MAP_ENTRY = Shuffleboard.getTab("Configuration")
            .add("Shooter Speed Map", new double[] {
                    1.0, 1630.0,
                    2.1, 1630.0,
                    3.0, 1814.0,
                    4.0, 2022.0,
                    5.0, 2230.0,
                    6.0, 2438.0
            })
            .getEntry();

    private final GenericEntry IDLE_SPEED = Shuffleboard.getTab("Configuration")
            .add("IDLE SPEED", 4.9)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();

    private final GenericEntry FIRE_BOOST_VOLTAGE = Shuffleboard.getTab("Configuration")
            .add("FIRING BOOST VOLTAGE", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();

    private final GenericEntry SHOOTER_VEL = Shuffleboard.getTab("Configuration")
            .add("Shooter Velocity", -1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .getEntry();

    public Shooter() {
        shooterMotor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(37, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(58, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pidf(0.001, 0, 0.0001, 0.0003398478); // TODO: tune
        shooterConfig.smartCurrentLimit(40);
        shooterConfig.voltageCompensation(11);
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.velocityConversionFactor(1.0 / 2.0);
        encoderConfig.positionConversionFactor(1.0 / 2.0);
        shooterConfig.apply(encoderConfig);
        shooterMotor.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        shooterPID = shooterMotor.getClosedLoopController();

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(shooterMotor, true);
        followerConfig.smartCurrentLimit(40);
        followerConfig.voltageCompensation(12);
        followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        followerMotor.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);
        feederConfig.voltageCompensation(12);
        feederConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        this.setDefaultCommand(this.run(() -> {
            shooterMotor.setVoltage(0);
            setFiring(false);
        }));
    }

    @Override
    public void periodic() {
        double[] mapData = SPEED_MAP_ENTRY.getDoubleArray(new double[0]);
        if (!Arrays.equals(mapData, lastMapData)) {
            speedMap.clear();
            for (int i = 0; i < mapData.length - 1; i += 2) {
                speedMap.put(mapData[i], mapData[i + 1]);
            }
            lastMapData = mapData;
            System.out.println("Speed Map Updated!");
        }

        Logger.recordOutput("Shooter/Main Velocity", shooterMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Follower Velocity", followerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/Feeder Velocity", feederMotor.getEncoder().getVelocity());

        Logger.recordOutput("Shooter/Main Current", shooterMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Follower Current", followerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Feeder Current", feederMotor.getOutputCurrent());

        Logger.recordOutput("Shooter/Applied Output", shooterMotor.getAppliedOutput());

        Logger.recordOutput("Shooter/Firing", feederMotor.getAppliedOutput() > 0);

        Logger.recordOutput("Shooter/Idle Speed", IDLE_SPEED.getDouble(0));
        Logger.recordOutput("Shooter/Fire Boost Voltage", FIRE_BOOST_VOLTAGE.getDouble(0));
    }

    /**
     * @param speed       base speed
     * @param firingBoost Apply extra voltage when firing
     */
    public void setShooterSpeed(double speed, boolean firingBoost) {
        double ff = 0;
        if (firingBoost) {
            ff = FIRE_BOOST_VOLTAGE.getDouble(0);
        }

        if (Math.abs(speed) < 1) {
            shooterMotor.setVoltage(0);
        }

        var configValue = SHOOTER_VEL.getDouble(-1);
        if (configValue != -1) {
            speed = SHOOTER_VEL.getDouble(-1);
        }

        Logger.recordOutput("Shooter/Velocity Setpoint", speed);
        shooterPID.setReference(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                ff, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public boolean isAtSpeed(double speed) {
        var configValue = SHOOTER_VEL.getDouble(-1);
        if (configValue != -1) {
            speed = SHOOTER_VEL.getDouble(-1);
        }
        return Math.abs(getShooterVelocity() - speed) < SHOOTER_VELOCITY_RANGE;

    }

    private static final double FEEDER_FIRING_VOLTAGE = 8;

    public void setFiring(boolean isFiring) {
        feederMotor.setVoltage(isFiring ? FEEDER_FIRING_VOLTAGE : 0);
    }

    public double getShooterVelocity() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public double getWantedVelocity(double distance) {
        return speedMap.get(distance);
    }

    public double getFeederCurrent() {
        return feederMotor.getOutputCurrent();
    }
}
