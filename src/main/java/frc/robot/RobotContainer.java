// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.limelightPositionCom;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import com.reduxrobotics.canand.CanandEventLoop;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DispenserCommand;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.FunnelSub;
import frc.robot.subsystems.SwerveSub;
import swervelib.parser.SwerveParser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  SwerveDrive m_Swerve;

  private final FunnelSub FunnelSubSystem = new FunnelSub();

  private final SwerveSub subSwerve;
  private final LimelightSub m_LimelightSub = new LimelightSub();
  
 
  private final DriveCmd driveCom;
  private final IntakeCommand inCom; 
  private final DispenserCommand disCom;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandJoystick leftJoystick = new CommandJoystick(OperatorConstants.LeftJoystickPort);
  private final CommandJoystick rightJoystick = new CommandJoystick(OperatorConstants.RightJoystickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CanandEventLoop.getInstance();

    inCom = new IntakeCommand(FunnelSubSystem);
    disCom = new DispenserCommand(FunnelSubSystem);

    try {
      double maximumSpeed = Units.feetToMeters(4.5);
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_Swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("Failed");
      e.printStackTrace();
    }
    subSwerve = new SwerveSub(m_Swerve);
    driveCom = new DriveCmd(subSwerve, leftJoystick, rightJoystick);
    
    subSwerve.setDefaultCommand(driveCom);
    disCom = new DispenserCommand(FunnelSubSystem); 
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    leftJoystick.button(7).onTrue(new InstantCommand(subSwerve::zeroGyro, subSwerve));

    leftJoystick.button(3).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  true));
    rightJoystick.button(4).whileTrue(new limelightPositionCom(m_LimelightSub, subSwerve,  false));

    m_driverController.a().whileTrue(inCom);
    m_driverController.b().whileTrue(disCom);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(); // TODO: ADD A AUTO CMD
  }
}
