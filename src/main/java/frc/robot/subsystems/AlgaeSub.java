// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSub extends SubsystemBase {
  /** Creates a new AlgaeSub./**/
    public final class Arm{
     SparkMax armMotor; 
     
     public Arm(){
      armMotor = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
     }
     public void spin(double spin){
      armMotor.set(spin);
     }
     }
  
    public final class Rotater{
      SparkMax rotatorMotor;

      public Rotater(){
       rotatorMotor = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);
      }  
      public void spin(double spin){   
      rotatorMotor.set(spin); 
      }
      public void stop(){
        rotatorMotor.set(0);
      }
      }
    public final Rotater rotater;
    public final Arm arm;
  public AlgaeSub() {
    rotater = new Rotater();
    arm = new Arm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}

