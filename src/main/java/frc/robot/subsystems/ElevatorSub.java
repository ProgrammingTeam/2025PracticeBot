// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
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
      SparkMaxConfig config = new SparkMaxConfig();
      config.follow(21,true);
 //   rightElevateMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    


  }
  public void changePosition(ElevatorPositions position){
    m_Position = position;
    pid.setSetpoint(m_Position.height);
        switch (m_Position) {
          case L1: elevatorDriveSpeedMultiplier = 0.4;
        break;
      case L2: elevatorDriveSpeedMultiplier = 0.3;
        break;
      case L3: elevatorDriveSpeedMultiplier = 0.2;
        break;
      case L4: elevatorDriveSpeedMultiplier = 0.1;
        break;
      case corolStation: elevatorDriveSpeedMultiplier = 0.5;
        break;
      case travel: default:
        elevatorDriveSpeedMultiplier = 1;
        break;
      }
  }
 
    
 

  @Override
  public void periodic() {
   SmartDashboard.putNumber("encoder Position", leftEncoder.getPosition());
  move(MathUtil.clamp(pid.calculate(EncoderValue()), -0.2, 0.2));
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
  
}
