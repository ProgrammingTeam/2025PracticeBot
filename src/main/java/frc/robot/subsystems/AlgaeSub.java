// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSub extends SubsystemBase {

  // Creates a final nested class "Arm"
  public final class Arm {
    SparkMax armMotor; 

    // Arm constructor
    public Arm() {
      armMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    }

    // Method to set the speed for the arm motor
    public void spin(double spin) {
      armMotor.set(spin);
      }
  }

  // Nested class rotator
  public final class Rotater {
    SparkMax rotatorMotor;

    // Rotator constuctor
    public Rotater() {
      rotatorMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    }

    // Methods to set speed of motor and stop method if needed
    public void spin(double spin){   
      rotatorMotor.set(spin); 
    }
    
    public void stop() {
      rotatorMotor.set(0);
    }
  }

  // Makes new local, public instances of nested clases
  public final Rotater rotater;
  public final Arm arm;

  // Constructor for AlgaeSub -- Parent Class
  public AlgaeSub() {
    rotater = new Rotater();
    arm = new Arm();
  }

  @Override
  public void periodic() {
  // Runs every scheduler run
  }
}

