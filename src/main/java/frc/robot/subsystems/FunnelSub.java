// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSub extends SubsystemBase {
  // changed both protected class to public final,
  // until a mentor comes to change it back and teach us how to use it this is how
  // it will be used -Loki
  // Protected classes are not accessable from outside the package -- Similar use to public except a difference 
  // Refer to @link https://stackoverflow.com/questions/215497/what-is-the-difference-between-public-protected-package-private-and-private-in
  public final class Intake {
    SparkMax intakeMotor;

    public Intake() {
      intakeMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
    }

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

    public Dispenser() {
      leftMotor = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);
      rightMotor = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
    }

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

  // Creates a new funnel sub constructor
  public FunnelSub() {
    intake = new Intake();
    dispenser = new Dispenser();

  }

  @Override
  public void periodic() {
  }
}
