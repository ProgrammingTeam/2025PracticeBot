// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// battery 

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;

public class ElevatorSub extends SubsystemBase {
  private final PIDController pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private final SparkMax leftElevateMotor = new SparkMax(Constants.CANBus.lElevator, MotorType.kBrushless);
  private final SparkMax rightElevateMotor = new SparkMax(Constants.CANBus.rElevator, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder;

  private ElevatorPositions m_Position;
  public double elevatorDriveSpeedMultiplier;

  public ElevatorSub() { 
    leftEncoder = leftElevateMotor.getEncoder();
    SparkMaxConfig configL = new SparkMaxConfig();

    // negative percent output results in increased height when not inverted
    configL.inverted(true);

    SparkMaxConfig configR = new SparkMaxConfig();
    configR.follow(Constants.CANBus.lElevator,false);

    leftElevateMotor.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevateMotor.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    pid.setTolerance(0.5);
  }

  
  public void changePosition(ElevatorPositions position){
    m_Position = position;
    pid.setSetpoint(m_Position.height);
  }
 
    
 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Current Elevator Height", encoderValueAsFieldHeight());
    SmartDashboard.putNumber("PID Output", pid.calculate(encoderValueAsFieldHeight()));
   
    move(MathUtil.clamp(pid.calculate(encoderValueAsFieldHeight()), -1, 1));
   
    SmartDashboard.putNumber("PID P Value", pid.getP());
    SmartDashboard.putNumber("PID I Value", pid.getI());
    SmartDashboard.putNumber("PID D Value", pid.getD());
    
    
    if ((ElevatorPositions.L4.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.1;
    }
    else if ((ElevatorPositions.L3.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.2;
    }
    else if ((ElevatorPositions.L2.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.3;
    }
    else if ((ElevatorPositions.L1.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.4;
    }
    else if ((ElevatorPositions.travel.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 1;
    }
    else {
      elevatorDriveSpeedMultiplier = 1;
    }
  }

  public void move(double elevateSpeed) {
    leftElevateMotor.set(elevateSpeed);
}
  public double EncoderValue() {
      return leftEncoder.getPosition();
  }
  
  public boolean atPidSetpoint() {
    return pid.atSetpoint();
  }

  public void resetEncoder()
  {
    leftEncoder.setPosition(0);
  }

  /**
   * @return end of coral manipulator compared to ground level of the field in inches
   */
  public double encoderValueAsFieldHeight()
  {
    return leftEncoder.getPosition() / Constants.ElevatorConstants.rotationsPerInch + Constants.ElevatorConstants.mountingHeight;
  }
}
