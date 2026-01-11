package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Hopper extends SubsystemBase {

    private SparkMax sparkMax;

    public Hopper() {
        sparkMax = new SparkMax(40, MotorType.kBrushless);
    }

    
    public Command run() {
        return Commands.run(
            () -> sparkMax.setVoltage(1),
            this
        );
    }

    public Command stop() {
        return Commands.run(
            () -> sparkMax.setVoltage(0),
            this
        );
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}