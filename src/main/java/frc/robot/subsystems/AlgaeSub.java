// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSub extends SubsystemBase {
  /** Creates a new AlgaeSub./ **/
  public final class Arm {
    SparkMax armMotor;
    PIDController PID;
    RelativeEncoder angleEncoder;

    public Arm() {
      armMotor = new SparkMax(41, SparkLowLevel.MotorType.kBrushless);
      PID = new PIDController(Constants.AlgeaConstants.Kp, 0, 0);
      angleEncoder = armMotor.getEncoder();
      PID.setTolerance(0.02);
    }

    private double motorRotationsToArmRotations(double motor)
    {
      return motor;
    }

    //In full circles, from [0, 0.25] is the effective range
    public void setSetpoint(double newSetpoint)
    {
      PID.setSetpoint(newSetpoint);
    }

    public double getPIDOutput()
    {
      return PID.calculate(motorRotationsToArmRotations(angleEncoder.getPosition()));
    }
    

    protected void spin(double spin) {
      armMotor.set(spin);
    }
  }

  public final class Rotater {
    SparkMax rotatorMotor;

    public Rotater() {
      rotatorMotor = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    }

    public void spin(double spin) {
      rotatorMotor.set(spin);
    }

    public void stop() {
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
    double rotateSpeed = arm.getPIDOutput();
    rotateSpeed = MathUtil.clamp(rotateSpeed, -0.1, 0.1);
      arm.spin(rotateSpeed);
      SmartDashboard.putNumber("Encoder position", arm.angleEncoder.getPosition());  
    }
}
