package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

   

    private static final double INTAKE_VOLTAGE = 4.0;
    private static final double OUTTAKE_VOLTAGE = -4.0;
    private final SparkMax intakeSparkMax;
    private final DoubleSolenoid intakeSwitch;


    public Intake() {
        intakeSparkMax = new SparkMax(53, MotorType.kBrushless);
        SparkMax followerSparkMax = new SparkMax(30, MotorType.kBrushless);
        intakeSwitch = new DoubleSolenoid(8, PneumaticsModuleType.CTREPCM, 0, 1);
        intakeSparkMax.setVoltage(0);
        var intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.smartCurrentLimit(40);
        intakeConfig.voltageCompensation(12);
        intakeSparkMax.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.setDefaultCommand(this.run(() -> intakeSparkMax.setVoltage(0)));

        var followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).follow(53, true);
        followerSparkMax.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    
    public Command intakeDown() {
        return Commands.runOnce(() -> intakeSwitch.set(DoubleSolenoid.Value.kForward));
    }

    public Command intakeUp() {
        return Commands.runOnce(() -> intakeSwitch.set(DoubleSolenoid.Value.kReverse));
    }

    public Command toggleIntake() {
        return Commands.runOnce(intakeSwitch::toggle);
    }

    public boolean isExtended() {
        return (intakeSwitch.get().equals(DoubleSolenoid.Value.kForward));
    }


    public Command intake() {
        return Commands.run(() -> intakeSparkMax.setVoltage(INTAKE_VOLTAGE), this);
    }

    public Command outtake() {
        return Commands.run(() -> intakeSparkMax.setVoltage(OUTTAKE_VOLTAGE), this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake Current", intakeSparkMax.getOutputCurrent());
        Logger.recordOutput("Intake Output", intakeSparkMax.getAppliedOutput());

    }
    
}
