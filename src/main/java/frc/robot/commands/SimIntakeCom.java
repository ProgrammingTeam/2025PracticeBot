// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Simulation.IntakeIOSim;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimIntakeCom extends Command {
  private final IntakeIOSim m_IntakeIOSim;
  private final XboxController m_XboxController;
  boolean intakeRunning = false;
  /** Creates a new SimIntakeCom. */
  public SimIntakeCom(IntakeIOSim ioSim, XboxController xBox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_XboxController = xBox;
    m_IntakeIOSim = ioSim;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeIOSim.setRunning(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_XboxController.getLeftBumperButtonPressed()) {
      // toggles weather or not the intake is running
      intakeRunning =! intakeRunning;  
      m_IntakeIOSim.setRunning(intakeRunning);
    }
    
    if (m_XboxController.getRightBumperButtonPressed()) {
      // toggles weather or not the intake is running
      
      m_IntakeIOSim.launchCorral();
    }
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
