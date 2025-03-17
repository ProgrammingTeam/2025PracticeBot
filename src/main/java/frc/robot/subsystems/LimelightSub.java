// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

// Class of LimelightSub that holds pose related variables
public class LimelightSub extends SubsystemBase {
  public double distenceFromTarget;
  SwerveDrivePoseEstimator m_poseEstimator;
  Canandgyro m_gyro;
  boolean doRejectUpdate;
  boolean redAlliance;
  
  // The constructor of LimelightSub
  public LimelightSub(SwerveSub swerveSub) {
      m_poseEstimator = swerveSub.swerveDrive.swerveDrivePoseEstimator;
      m_gyro = (Canandgyro) swerveSub.swerveDrive.getGyro().getIMU();
  }

  @Override
  public void periodic() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      redAlliance = true;
    } else {
      redAlliance = false;
    }
  
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = redAlliance ? 
            LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight") : 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    doRejectUpdate = false;
    
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    if(Math.abs(m_gyro.getAngularVelocityYaw()) > 720) {
      doRejectUpdate = true;
    }
    
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    
    if(!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
    }  
  }
}
