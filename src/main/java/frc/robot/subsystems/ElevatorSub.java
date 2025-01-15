// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new ElevatorSub. */
    SparkMax leftElevateMotor = new SparkMax(11, null);
    SparkMax rightElevateMotor = new SparkMax(12, null);
  public ElevatorSub() {

    SparkMaxConfig config = new SparkMaxConfig();
    
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
