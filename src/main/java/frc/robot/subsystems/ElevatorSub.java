// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// battery 

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

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {

  /** Creates a new ElevatorSub. */
    SparkMax leftElevateMotor = new SparkMax(21, MotorType.kBrushless);
    SparkMax rightElevateMotor = new SparkMax(22, MotorType.kBrushless);
    RelativeEncoder leftEncoder;
    public double elevatorDriveSpeedMultiplier;
  public ElevatorSub() { 
    leftEncoder = leftElevateMotor.getEncoder();
    SparkMaxConfig configL = new SparkMaxConfig();

    // when inverted, positive percent output drives the elevator upward
    configL.inverted(true);

    SparkMaxConfig configR = new SparkMaxConfig();
    configR.follow(21,false);

    leftElevateMotor.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevateMotor.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
