package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Hopper extends SubsystemBase {

    private SparkMax sparkMax;
    private SparkMaxConfig sparkMaxConfig;

    public Hopper() {
        sparkMax = new SparkMax(40, MotorType.kBrushless);
        sparkMaxConfig = new SparkMaxConfig();
        
        sparkMaxConfig
                .idleMode(IdleMode.kBrake)
                //.inverted(True)
                .smartCurrentLimit(40)
                .voltageCompensation(12);
                
        sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public Command runHopper() {
        return Commands.runOnce(
            () -> sparkMax.setVoltage(1),
            this
        );
    }

    public Command stopHopper() {
        return Commands.runOnce(
            () -> sparkMax.setVoltage(0),
            this
        );
    }
}