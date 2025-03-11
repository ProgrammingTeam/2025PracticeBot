// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCmd extends Command {
  ElevatorSub M_Elvsub;
  CommandXboxController M_xboxController;
  CommandJoystick m_Joystick;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(ElevatorSub Elvsub, CommandXboxController xboxController, CommandJoystick joystick) {
    M_Elvsub = Elvsub;
    M_xboxController = xboxController;
    m_Joystick = joystick;
    addRequirements(Elvsub); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    M_Elvsub.move(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    M_Elvsub.move(M_xboxController.getLeftY() * m_Joystick.getRawAxis(3));
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
