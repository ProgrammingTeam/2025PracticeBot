// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSub extends SubsystemBase {
  public double distenceFromTarget;
  private double opposite;
  private double TxValue;
  private double TyValue;
  private int TidValue;
  private NetworkTableEntry LimeTX;
  private NetworkTableEntry LimeTY;
  private NetworkTableEntry LimeTid;
  /** Creates a new LimelightSub. */
  public LimelightSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
