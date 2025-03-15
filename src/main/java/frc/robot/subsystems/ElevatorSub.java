// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// battery 

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;

public class ElevatorSub extends SubsystemBase {
  private final PIDController pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private ElevatorPositions m_Position;
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

  
  public void changePosition(ElevatorPositions position){
    m_Position = position;
    pid.setSetpoint(m_Position.height);
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
    else {elevatorDriveSpeedMultiplier = 1;
    }
  }
 
    
 

  @Override
  public void periodic() {
   SmartDashboard.putNumber("encoder Position", leftEncoder.getPosition());
  move(MathUtil.clamp(pid.calculate(EncoderValue()), -1, 1));
  SmartDashboard.putNumber("PID P Value", pid.getP());
  SmartDashboard.putNumber("PID I Value", pid.getI());
  SmartDashboard.putNumber("PID D Value", pid.getD());
  SmartDashboard.putNumber("Current PID calculation", pid.calculate(EncoderValue()));
    // This method will be called once per scheduler run
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
}
