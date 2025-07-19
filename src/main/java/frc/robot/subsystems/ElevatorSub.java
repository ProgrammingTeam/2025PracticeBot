// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Percent;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.thethriftybot.ThriftyNova.PIDConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;

public class ElevatorSub extends SubsystemBase {
  
  
 // private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0625, 0, 0,0);

  
  private SparkMax leftElevateMotor = new SparkMax(Constants.CANBus.lElevator, MotorType.kBrushless);
  private SparkMax rightElevateMotor = new SparkMax(Constants.CANBus.rElevator, MotorType.kBrushless);

  private SparkClosedLoopController pid = leftElevateMotor.getClosedLoopController();

  private final RelativeEncoder leftEncoder;

  private double m_Position;
  public double elevatorDriveSpeedMultiplier;
  
  // Constructor of ElevatorSub
  public ElevatorSub() { 
    leftEncoder = leftElevateMotor.getEncoder();
    SparkMaxConfig configL = new SparkMaxConfig();
    // Set PID gains
    configL.closedLoop
      .p(0.4)
      .i(Constants.ElevatorConstants.kI)
      .d(Constants.ElevatorConstants.kD)
      .outputRange(Constants.ElevatorConstants.minOutput, Constants.ElevatorConstants.maxOutput)
      .maxMotion
      .maxVelocity(Constants.ElevatorConstants.maxVel)
      .maxAcceleration(Constants.ElevatorConstants.maxAccel)
      .allowedClosedLoopError(Constants.ElevatorConstants.allowedErr);

    // negative percent output results in increased height when not inverted
    configL.inverted(true);

    SparkMaxConfig configR = new SparkMaxConfig();
    configR.follow(Constants.CANBus.lElevator,true);

    configL.smartCurrentLimit(30);
    configR.smartCurrentLimit(30);

    leftElevateMotor.configure(configL, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevateMotor.configure(configR, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
// config.closedLoop.maxMotion
//     .maxVelocity(Constants.ElevatorConstants.maxVelocity)
//     .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
//     .allowedClosedLoopError(Constants.ElevatorConstants.acceptibleErrorZone);
  }
 
  public void changePosition(double position){
    m_Position = position;
    pid.setReference(position, ControlType.kMAXMotionPositionControl);
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator percent Output", leftElevateMotor.get());
    // SmartDashboard.putNumber("Current Elevator Height", encoderValueAsFieldHeight());
   // SmartDashboard.putNumber("PID Output", pid.calculate(leftEncoder.getPosition()));
    
   // SmartDashboard.putNumber("Elevator OutPut", pid.calculate(leftEncoder.getPosition())); // + (elevatorFeedforward.calculate(leftEncoder.getVelocity()) / Constants.voltageSupply));


    SmartDashboard.putNumber("PID set point", m_Position);

   // move(MathUtil.clamp(pid.calculate(leftEncoder.getPosition()), -1, 1)); // + (elevatorFeedforward.calculate(leftEncoder.getVelocity()) / Constants.voltageSupply), -1, 1));
   
    SmartDashboard.putNumber("PID P Value", 1);
    SmartDashboard.putNumber("PID I Value", Constants.ElevatorConstants.kI);
    SmartDashboard.putNumber("PID D Value", Constants.ElevatorConstants.kD);
    
    SmartDashboard.putNumber("Elevator Velocity", leftEncoder.getVelocity());
    if ((ElevatorPositions.L4.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.1;
    } else if ((ElevatorPositions.L3.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 0.2;
    } else if ((ElevatorPositions.travel.height <= leftEncoder.getPosition())) {
      elevatorDriveSpeedMultiplier = 1;
    } else {
      elevatorDriveSpeedMultiplier = 1;
    }
  }

 // public void move(double elevateSpeed) {
//  leftElevateMotor.set(elevateSpeed);
 // }
  
  public double EncoderValue() {
    return leftEncoder.getPosition();
  }
  
  public void resetEncoder() {
    leftEncoder.setPosition(0);
  }

  public double currentSetPoint() {
    // 
    return m_Position;
  }

  //@return end of coral manipulator compared to ground level of the field in inches
  // public double encoderValueAsFieldHeight() {
  //   return leftEncoder.getPosition() / Constants.ElevatorConstants.rotationsPerInch + Constants.ElevatorConstants.mountingHeight;
  // }
}
