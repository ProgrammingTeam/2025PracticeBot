// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Kilo;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new ElevatorSub. */
    SparkMax leftElevateMotor = new SparkMax(11, MotorType.kBrushless);
    SparkMax rightElevateMotor = new SparkMax(12, MotorType.kBrushless);
    SparkClosedLoopController m_PIDController = leftElevateMotor.getClosedLoopController();
    
  public ElevatorSub() {
    
    m_PIDController.setReference(0.6, SparkBase.ControlType.kPosition);
    SparkMaxConfig config = new SparkMaxConfig();
    // Set PID gains
    config.closedLoop
      .p(Constants.ElevatorConstants.kP)
      .i(Constants.ElevatorConstants.kI)
      .d(Constants.ElevatorConstants.kD)
      .outputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);
    // Set kFF
    config.closedLoop.velocityFF(1/Constants.ElevatorConstants.kV);
    // Set MAXMotion parameters
    config.closedLoop.maxMotion
      .maxVelocity(Constants.ElevatorConstants.maxVel)
      .maxAcceleration(Constants.ElevatorConstants.maxAccel)
      .allowedClosedLoopError(Constants.ElevatorConstants.allowedErr);

    config.follow(11,true);
    rightElevateMotor.configure(config, null, null);
    //rightElevateMotor
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void move(double elevateSpeed) {
    leftElevateMotor.set(elevateSpeed);
}
}
