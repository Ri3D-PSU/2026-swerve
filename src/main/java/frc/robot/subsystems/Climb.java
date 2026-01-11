// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private SparkMax climbSparkMaxLeft;
  private final SparkClosedLoopController leftPID;

  private SparkMax climbSparkMaxRight;

  // private ElevatorFeedforward elevatorFF;
  // private ArmFeedforward armFF;
  // private double ARM_ANGLE_OFFSET = Math.PI / 2;

  private final double CLIMB_RATIO = 1; // This is either the rotation or height of the arm depending on which gets chosen

  /** Creates a new Climb. */
  public Climb() {
    climbSparkMaxLeft = new SparkMax(50, MotorType.kBrushless);
    leftPID = climbSparkMaxLeft.getClosedLoopController();
    climbSparkMaxRight = new SparkMax(51, MotorType.kBrushless);

    // elevatorFF = new ElevatorFeedforward(0, 0, 0, 0);
    // armFF = new ArmFeedforward(0, 0, 0, 0);
    
    SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12);
    
    SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
                .idleMode(IdleMode.kBrake)
                .follow(climbSparkMaxLeft, true)
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

  public Command setVoltage(double volts) {
    return Commands.runOnce(
      () -> {climbSparkMaxLeft.setVoltage(volts);}, 
      this);
  }

  public Command setPosition(double height) { // height or angle
    return Commands.runOnce(
      () -> {
        double FF = 0;
        // double FF = elevatorFF.calculate(0);
        // double FF = armFF.calculate(climbSparkMaxLeft.getEncoder().getPosition() + ARM_ANGLE_OFFSET, 0);
        leftPID.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF);}, 
      this);
  }
}