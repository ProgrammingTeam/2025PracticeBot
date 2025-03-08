// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimulateCoral extends Command {
  private final SwerveDriveSimulation m_mapleSimDrive;
  /** Creates a new ArmCmd. */
  public SimulateCoral(SwerveSub swerve) {
    if (swerve.getMapleSimDrive().isPresent()) 
      m_mapleSimDrive = swerve.getMapleSimDrive().get();
    else m_mapleSimDrive = null;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        m_mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0),
        // Obtain robot speed from drive simulation
        m_mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        m_mapleSimDrive.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        Meters.of(1.28),
        // The initial speed of the coral
        MetersPerSecond.of(2),
        // The coral is ejected at a 35-degree slope
        Degrees.of(-35)));
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
