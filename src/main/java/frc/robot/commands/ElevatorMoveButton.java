// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Alex was here, 1/14/2025

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveButton extends Command {
  private final CommandXboxController m_Controller;
  private final ElevatorSub m_ElevatorSub;
  /** Creates a new ElevatorMoveButton. */
  private final PIDController pid = new PIDController(kP, kI, kD);
  public ElevatorMoveButton(ElevatorSub elevatorSub, CommandXboxController controller) {
    m_ElevatorSub = elevatorSub;
    m_Controller = controller;
    addRequirements(m_ElevatorSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSub.move(0);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSub.move(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
