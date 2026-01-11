package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {

   

    private static final double INTAKE_VOLTAGE = 4.0;
    private static final double OUTTAKE_VOLTAGE = -4.0;
    private SparkMax IntakeSparkMax;
    private SparkMax bottomSparkMax;
    //private Solenoid IntakeSwitch;

    public Intake() {
        IntakeSparkMax = new SparkMax(36, MotorType.kBrushless);
        bottomSparkMax = new SparkMax(37, MotorType.kBrushless);
        //IntakeSwitch = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
        IntakeSparkMax.setVoltage(0);
        bottomSparkMax.setVoltage(0);
    }
//
//    public Command IntakeDown() {
//        return Commands.runOnce(() -> IntakeSwitch.set(true), this);
//    }
//
//    public Command IntakeUp() {
//        return Commands.runOnce(() -> IntakeSwitch.set(false), this);
//    }
//
//    public Command toggle() {
//        return Commands.runOnce(() -> IntakeSwitch.toggle(), this);
//    }
//
//    public boolean isExtended() {
//        return IntakeSwitch.get();
//    }

    public void intake() {
        IntakeSparkMax.set(-1.0); bottomSparkMax.set(0.2);
    }
    public void stop() {IntakeSparkMax.set(0); bottomSparkMax.set(0);}

    public void outtake() {
        IntakeSparkMax.setVoltage(OUTTAKE_VOLTAGE);
    }


    

    
    
    
    
}
