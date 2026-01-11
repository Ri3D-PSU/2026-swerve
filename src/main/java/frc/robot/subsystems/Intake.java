package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {

   

    private static final double INTAKE_VOLTAGE = 4.0;
    private static final double OUTTAKE_VOLTAGE = -4.0;
    private SparkMax IntakeSparkMax;
    private Solenoid IntakeSwitch;
    private SparkMaxConfig IntakeConfig;

    public Intake() {
        IntakeSparkMax = new SparkMax(36, MotorType.kBrushless);
        IntakeSwitch = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);    
        IntakeSparkMax.setVoltage(0);
        IntakeConfig.idleMode(IdleMode.kBrake);       


    }
    
    public Command IntakeDown() {
        return Commands.runOnce(() -> IntakeSwitch.set(true), this);
    }

    public Command IntakeUp() {
        return Commands.runOnce(() -> IntakeSwitch.set(false), this);
    }
    
    public Command toggle() {
        return Commands.runOnce(() -> IntakeSwitch.toggle(), this);
    }

    public boolean isExtended() {
        return IntakeSwitch.get();
    }

    public void intake() {
        IntakeSparkMax.setVoltage(INTAKE_VOLTAGE);
    }

    public void outtake() {
        IntakeSparkMax.setVoltage(OUTTAKE_VOLTAGE)
    }


    

    
    
    
    
}
