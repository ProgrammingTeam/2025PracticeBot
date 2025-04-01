// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSub;

// Class of IntakeCommand -- Command
public class IntakeCommand extends Command {
  FunnelSub m_FunnelSub;

  // Constructor of IntakeCommand
  public IntakeCommand(FunnelSub subFunnel) {
    m_FunnelSub = subFunnel;
    
    addRequirements(m_FunnelSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FunnelSub.intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_FunnelSub.intake.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_FunnelSub.intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
