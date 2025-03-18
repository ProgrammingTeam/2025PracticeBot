// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSub extends SubsystemBase {
  public double distenceFromTarget;
  //public double VerticleOffsetFromTarget;
  private double heightDifOfLimeLightToTag;
  
  private double TyValue;
  private double TxValue;
  private double horrizontalOffsetFromTag;
  private long tagIDNumber;
  private NetworkTableEntry LimeTY;
  private NetworkTableEntry LimeTX;
  private NetworkTableEntry LimeTid;
  
  // Constructor for Limelight sub
  public LimelightSub() {
    try {
      NetworkTable Limelight = NetworkTableInstance.getDefault().getTable("limelight");
      LimeTY = Limelight.getEntry("ty");
      LimeTX = Limelight.getEntry("tx");
      LimeTid = Limelight.getEntry("tid");

    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("Limelight has failed init.");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    try {
      TyValue = LimeTY.getDouble(0) +2;
      TxValue = LimeTX.getDouble(0); 
      tagIDNumber = LimeTid.getInteger(0);  

      heightDifOfLimeLightToTag = Constants.LimelightConstants.targetHeights[(int)tagIDNumber] - Constants.LimelightConstants.limelightHeight;
      distenceFromTarget = heightDifOfLimeLightToTag / 
       Math.tan(Math.toRadians(TyValue + Constants.LimelightConstants.angleOffset));
 
      horrizontalOffsetFromTag = distenceFromTarget * Math.tan(Math.toRadians(TxValue));

      SmartDashboard.putNumber("txValue", TxValue);
      SmartDashboard.putNumber("tyValue", TyValue);
      SmartDashboard.putNumber("distence From Target", distenceFromTarget);
      SmartDashboard.putNumber("verticleOffsetFromTag", horrizontalOffsetFromTag);
    } catch (Exception e) {
    // TODO: handle exception
    }
  }
  
  public double getVerticleDist() {
      return horrizontalOffsetFromTag;
    }
}
