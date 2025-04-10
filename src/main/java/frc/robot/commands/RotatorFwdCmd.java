// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSub;

// The class for RotatorForwardCommand
public class RotatorFwdCmd extends Command {
  AlgaeSub m_AlgaeSub;
  public RotatorFwdCmd(AlgaeSub SubAlgea) {
    m_AlgaeSub = SubAlgea;
    
    addRequirements(m_AlgaeSub);
  }

  // Called during initialization
  @Override
  public void initialize() {
    m_AlgaeSub.rotater.spin(0.0);
  }

  // Called every time this cmd is scheduled
  @Override
  public void execute() {
    m_AlgaeSub.rotater.spin(1);
  }

  // Called once when the cmd is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeSub.rotater.spin(0.0);
  }

  // Returns false when cmd is finished
  @Override
  public boolean isFinished() {
    return false;
  }
}
