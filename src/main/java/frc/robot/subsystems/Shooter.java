package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase  {

    private SparkMax shooterMotor;
    private SparkMax followerMotor;

    private SparkMax feederMotor;

    public Shooter() {
        shooterMotor = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop.pidf(0.1, 0, 0, 0.1); // TODO: tune
        shooterConfig.smartCurrentLimit(50);
        shooterConfig.voltageCompensation(12);
        shooterMotor.configure(shooterConfig,  SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(shooterMotor); // TODO: check invert
        followerConfig.smartCurrentLimit(50);
        followerConfig.voltageCompensation(12);
        followerMotor.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(30);
        feederConfig.voltageCompensation(12);
        feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.setDefaultCommand(setShooterSpeed(0));
    }



    public Command setShooterSpeed(double speed) {
        return Commands.run(
                () -> shooterMotor.set(speed), this
        );
    }


}
