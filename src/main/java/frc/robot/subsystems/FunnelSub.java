// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSub extends SubsystemBase {
  // Changed both protected class to public final,
  // until a mentor comes to change it back and teach us how to use it this is how
  // it will be used -Loki
  
  // Protected makes only the package owners accesible to this class;
  // this is purely java's purpose for protected
  // It's similar to public expect no outside packages
  public final class Intake {
    SparkMax intakeMotor;

    // Constructor for Intake
    public Intake() {
      intakeMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
    }

    // Basic functions for intake
    public void forward() {
      intakeMotor.set(0.2);
    } 

    public void stop() {
      intakeMotor.set(0);
    }
  }

  public final class Dispenser {
    SparkMax leftMotor;
    SparkMax rightMotor;

    // Constructor for dispenser
    public Dispenser() {
      leftMotor = new SparkMax(21, SparkLowLevel.MotorType.kBrushless);
      rightMotor = new SparkMax(22, SparkLowLevel.MotorType.kBrushless);
    }

    // Basic Functions
    public void forward() {
      leftMotor.set(0.2);
      rightMotor.set(0.2);
    }

    public void stop() {
      leftMotor.set(0);
      rightMotor.set(0);
    }
  }

  public final Intake intake;
  public final Dispenser dispenser;

  // Constructor for FunnelSub
  public FunnelSub() {
    intake = new Intake();
    dispenser = new Dispenser();
  }

  @Override
  public void periodic() {
    
  }
}
