// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//alex was here

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSub;

// Class of AutoElevatorCmd
public class AutoElevatorCmd extends Command {
  private final ElevatorSub m_ElvSub;
  private final ElevatorPositions m_ElevatorPosition;
  
  // Constructor for AutoElevatorCmd
  public AutoElevatorCmd(ElevatorSub ElvSub, ElevatorPositions elevatorPosition) {
    m_ElvSub = ElvSub;
    m_ElevatorPosition = elevatorPosition;
    addRequirements(m_ElvSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when first init
  @Override
  public void initialize() {
    m_ElvSub.changePosition(m_ElevatorPosition);
  }

  // TODO: Add logic to execute
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ElvSub.atPidSetpoint(); 
  }
}
