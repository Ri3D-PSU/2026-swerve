// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private SparkMax climbSparkMaxLeft;
  private final SparkClosedLoopController leftPID;

  private SparkMax climbSparkMaxRight;

  private ArmFeedforward armFF;

  private final double CLIMB_RATIO = 180; // This is either the rotation or height of the arm depending on which gets chosen

  private DoubleSolenoid extensionSolenoid;

  /** Creates a new Climb. */
  public Climb() {
    climbSparkMaxLeft = new SparkMax(50, MotorType.kBrushless);
    leftPID = climbSparkMaxLeft.getClosedLoopController();
    climbSparkMaxRight = new SparkMax(51, MotorType.kBrushless);

    armFF = new ArmFeedforward(0, 0, 0, 0);

    extensionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // TODO: get correct channels
    
    SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12);
    
    SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
                .idleMode(IdleMode.kBrake)
                .follow(climbSparkMaxLeft, false)
                .smartCurrentLimit(40)
                .voltageCompensation(12);

    leftConfig.closedLoop
        .pidf(0.5, 0, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);

    leftConfig.encoder
            .positionConversionFactor(1. / CLIMB_RATIO)
            .velocityConversionFactor(1. / CLIMB_RATIO / 60.);

    climbSparkMaxLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbSparkMaxRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setVoltageWithFeedforward(double volts, double currentAngle) {
    return Commands.runOnce(
      () -> {
        double FF = armFF.calculate(currentAngle, 0);
        climbSparkMaxLeft.setVoltage(volts + FF);
      }, 
      this);
  }

  public void fixPIDPositionReference(double currentAngle) { // height or angle
     // Because feedforward is continuously calculated
        double FF = armFF.calculate(currentAngle, 0);
        leftPID.setReference(climbSparkMaxLeft.getEncoder().getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);
  }

  public Command extend(boolean retract) {
      return Commands.runOnce(() -> {
        if(retract) {
          extensionSolenoid.set(Value.kReverse);
          Logger.recordOutput("Climber/Solenoid Status", "Retract");
        } else {
          extensionSolenoid.set(Value.kForward);
          Logger.recordOutput("Climber/Solenoid Status", "Extend");
        }
      });
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/Left Motor Current", climbSparkMaxLeft.getOutputCurrent());
    Logger.recordOutput("Climber/Right Motor Current", climbSparkMaxRight.getOutputCurrent());
    Logger.recordOutput("Climber/Left Motor Applied Output", climbSparkMaxLeft.getAppliedOutput());
    Logger.recordOutput("Climber/Right Motor Applied Output", climbSparkMaxRight.getAppliedOutput());
  }
}