// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSub;

public class DispenserCommand extends Command {
  FunnelSub m_FunnelSub;

  /** Creates a new DispenserCommand. */
  public DispenserCommand(FunnelSub funnel) {
    m_FunnelSub = funnel;
    addRequirements(m_FunnelSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FunnelSub.dispenser.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_FunnelSub.dispenser.forward();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
