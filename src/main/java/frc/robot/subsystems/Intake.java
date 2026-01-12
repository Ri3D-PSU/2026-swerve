package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

   

    private static final double INTAKE_VOLTAGE = 4.0;
    private static final double OUTTAKE_VOLTAGE = -4.0;
    private SparkMax IntakeSparkMax;
    private DoubleSolenoid IntakeSwitch;
    private SparkMaxConfig IntakeConfig;

    public Intake() {
        IntakeSparkMax = new SparkMax(36, MotorType.kBrushless);
        IntakeSwitch = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
        IntakeSparkMax.setVoltage(0);
        IntakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);     


    }
    
    public Command IntakeDown() {
        return Commands.runOnce(() -> IntakeSwitch.set(DoubleSolenoid.Value.kForward), this);
    }

    public Command IntakeUp() {
        return Commands.runOnce(() -> IntakeSwitch.set(DoubleSolenoid.Value.kReverse), this);
    }
    
    public Command toggle() {
        return Commands.runOnce(() -> IntakeSwitch.toggle(), this);
    }

    public boolean isExtended() {
        return (IntakeSwitch.get().equals(DoubleSolenoid.Value.kForward));
    }

    public void intake() {
        IntakeSparkMax.setVoltage(INTAKE_VOLTAGE);
    }

    public void outtake() {
        IntakeSparkMax.setVoltage(OUTTAKE_VOLTAGE);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake Current", IntakeSparkMax.getOutputCurrent());
        Logger.recordOutput("Intake Output", IntakeSparkMax.getAppliedOutput());

    }
    
}
