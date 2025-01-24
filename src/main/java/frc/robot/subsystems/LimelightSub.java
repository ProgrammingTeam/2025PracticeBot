// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSub extends SubsystemBase {

  public double distanceFromTarget;
  // public double VerticleOffsetFromTarget;
  private double heightDifOfLimeLightFrom;
  private double opposite;
  private double TyValue;
  private int TidValue;
  private NetworkTableEntry LimeTY;
  private NetworkTableEntry LimeTid;

  // Creates a new LimelightSub.
  public LimelightSub() {
    NetworkTable Limelight = NetworkTableInstance.getDefault().getTable("limelight");
    LimeTY = Limelight.getEntry("ty");
    LimeTid = Limelight.getEntry("tid");

    opposite = Constants.LimelightConstants.targetHeights[TidValue] - Constants.LimelightConstants.limelightHeight;
    distanceFromTarget = opposite / Math.tan
    (Math.toRadians(LimeTY.getDouble(0) + Constants.LimelightConstants.angleOffset));

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
