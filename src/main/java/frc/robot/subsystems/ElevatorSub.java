// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  //TODO: REMOVE BACK AND FRONT MOTOR LOGIC -- FIX LOGIC AND IMPLEMENT FEATURES
  /** Creates a new ElevatorSub. */
  private SparkMax motor1 = new SparkMax(24, MotorType.kBrushless);
  private SparkMax motor2 = new SparkMax(25, MotorType.kBrushless);
  private SparkMax motor3 = new SparkMax(26, MotorType.kBrushless);
  private SparkMax motor4 = new SparkMax(27, MotorType.kBrushless);
  public double elevatorDriveSpeedMultiplier;
  public SparkAbsoluteEncoder motor1Encoder = motor1.getAbsoluteEncoder();
  public SparkAbsoluteEncoder motor3Encoder = motor3.getAbsoluteEncoder();

  private double motor1_E_Val;
  private double motor3_E_Val;

  public ElevatorSub() {
  }

  public void frontMotors(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void backMotors(double speed) {
    motor3.set(speed);
    motor4.set(speed);
  }

  @Override
  public void periodic() {
    motor1_E_Val = motor1Encoder.getPosition();
    motor3_E_Val = motor3Encoder.getPosition();
    if (motor1_E_Val >= 20) {
      elevatorDriveSpeedMultiplier = 0.5;
    }
    else if (motor1_E_Val >= 10) {
      elevatorDriveSpeedMultiplier = 0.75;
    }
    else {
      elevatorDriveSpeedMultiplier = 1;
    }
    SmartDashboard.putNumber("Front distance", motor1_E_Val);
    SmartDashboard.putNumber("Back distance", motor3_E_Val);
  }
}
