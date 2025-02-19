// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Alex was here, 1/14/2025

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveButton extends Command {
  private final ElevatorSub m_ElevatorSub;
  private final ElevatorPositions m_Position;
  /** Creates a new ElevatorMoveButton. */
  private final PIDController pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  public ElevatorMoveButton(ElevatorSub elevatorSub, ElevatorPositions position) {
    m_ElevatorSub = elevatorSub;
    m_Position = position;
    addRequirements(m_ElevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSub.move(0);
    pid.setSetpoint(m_Position.height);
    switch (m_Position) {
      case L1: m_ElevatorSub.elevatorDriveSpeedMultiplier = 0.4;
        break;
      case L2: m_ElevatorSub.elevatorDriveSpeedMultiplier = 0.3;
        break;
      case L3: m_ElevatorSub.elevatorDriveSpeedMultiplier = 0.2;
        break;
      case L4: m_ElevatorSub.elevatorDriveSpeedMultiplier = 0.1;
        break;
      case corolStation: m_ElevatorSub.elevatorDriveSpeedMultiplier = 0.5;
        break;
      case travel: default:
        m_ElevatorSub.elevatorDriveSpeedMultiplier = 1;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ElevatorSub.move(MathUtil.clamp(pid.calculate(m_ElevatorSub.EncoderValue()), -0.2, 0.2));
    SmartDashboard.putNumber("PID P Value", pid.getP());
    SmartDashboard.putNumber("PID I Value", pid.getI());
    SmartDashboard.putNumber("PID D Value", pid.getD());
    SmartDashboard.putNumber("Current PID calculation", pid.calculate(m_ElevatorSub.EncoderValue()));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSub.move(0);
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
