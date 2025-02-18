// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import static edu.wpi.first.units.Units.Kilo;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new ElevatorSub. */
    SparkMax leftElevateMotor = new SparkMax(11, MotorType.kBrushless);
   // SparkMax rightElevateMotor = new SparkMax(12, MotorType.kBrushless);
    RelativeEncoder leftEncoder;
  public ElevatorSub() { 
    leftEncoder = leftElevateMotor.getEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(11,true);
 //   rightElevateMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder Position", leftEncoder.getPosition());
    // This method will be called once per scheduler run
  }
  public void move(double elevateSpeed) {
    leftElevateMotor.set(elevateSpeed);
}
  public double EncoderValue() {
      return leftEncoder.getPosition();
    
  }
}
