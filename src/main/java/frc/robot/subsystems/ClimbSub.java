// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSub extends SubsystemBase {
  private final SparkMax climbMotor = new SparkMax(32, MotorType.kBrushless);
  /** Creates a new ClimbSub. */
  public ClimbSub() {
   
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void climb(double climbSpeed) {
    climbMotor.set(climbSpeed);
  }
}


