// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  AbstractDriveTrainSimulation swerveDriveSimulation;

  
  private final RobotContainer m_robotContainer;
  StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("CoralPosesArray", Pose3d.struct)
  .publish();
  StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
  .getStructArrayTopic("AlgaePosesArray", Pose3d.struct)
  .publish();
  
  final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(COTS.ofMark4(
                  DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                  DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                  COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                  3)) // L3 Gear ratio
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(30), Inches.of(30)
          );
            
              
              /**
               * This function is run when the robot is first started up and should be used for any
               * initialization code.
               */
              public Robot() {
                // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
                // autonomous chooser on the dashboard.
                m_robotContainer = new RobotContainer();
              }
            
              /**
               * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
               * that you want ran during disabled, autonomous, teleoperated and test.
               *
               * <p>This runs after the mode specific periodic functions, but before LiveWindow and
               * SmartDashboard integrated updating.
               */
              @Override
              public void robotPeriodic() {
                // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
                // commands, running already-scheduled commands, removing finished or interrupted commands,
                // and running subsystem periodic() methods.  This must be called from the robot's periodic
                // block in order for anything in the Command-based framework to work.
                CommandScheduler.getInstance().run();
              }
            
              /** This function is called once each time the robot enters Disabled mode. */
              @Override
              public void disabledInit() {}
            
              @Override
              public void disabledPeriodic() {}
            
              /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
              @Override
              public void autonomousInit() {
                m_autonomousCommand = m_robotContainer.getAutonomousCommand();
            
                // schedule the autonomous command (example)
                if (m_autonomousCommand != null) {
                  m_autonomousCommand.schedule();
                }
              }
            
              /** This function is called periodically during autonomous. */
              @Override
              public void autonomousPeriodic() {}
            
              @Override
              public void teleopInit() {
                CameraServer.startAutomaticCapture("Drive CAM", 0);
                // This makes sure that the autonomous stops running when
                // teleop starts running. If you want the autonomous to
                // continue until interrupted by another command, remove
                // this line or comment it out.
                if (m_autonomousCommand != null) {
                  m_autonomousCommand.cancel();
                }
              }
            
              /** This function is called periodically during operator control. */
              @Override
              public void teleopPeriodic() {}
            
              @Override
              public void testInit() {
                // Cancels all running commands at the start of test mode.
                CommandScheduler.getInstance().cancelAll();
              }
            
            
            
              /** This function is called periodically during test mode. */
              @Override
              public void testPeriodic() {}
            
              /** This function is called once when the robot is first started up. */
              @Override
              public void simulationInit() {
                this.swerveDriveSimulation =

                new SwerveDriveSimulation(
                  // Specify Configuration
                  driveTrainSimulationConfig,
                  // Specify starting pose
                  new Pose2d(3, 3, new Rotation2d())
          );
                // Obtains the default instance of the simulation world, which is a Crescendo Arena.
            
            // Overrides the default simulation
            
              // Get the positions of the notes (both on the field and in the air)
              Pose3d[] coralPoses = SimulatedArena.getInstance()
              .getGamePiecesArrayByType("Coral");
            // Publish to telemetry using AdvantageKit
            Logger.recordOutput("FieldSimulation/CoralsPositions", coralPoses);
            
            Pose3d[] algaePoses = SimulatedArena.getInstance()
              .getGamePiecesArrayByType("Coral");
            // Publish to telemetry using AdvantageKit
            Logger.recordOutput("FieldSimulation/AlgaePositions", algaePoses);
            
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                  // We must specify a heading since the coral is a tube
                  new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
                  SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
                  SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(3,2)));
            
            
          
          }
            
          
            /** This function is called periodically whilst in simulation. */
            @Override
            public void simulationPeriodic() {
             SimulatedArena.getInstance().simulationPeriodic();
            coralPoses.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
            algaePoses.accept(SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            

//SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

  }
  
}
