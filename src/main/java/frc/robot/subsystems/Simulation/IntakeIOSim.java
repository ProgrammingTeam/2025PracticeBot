// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.SwerveSub;


/** Add your docs here. */
public class IntakeIOSim {
    //may need to come back laster to add values to shooter angle and LinearVel for proper shooting
    private Angle shooterAngle;
    private LinearVelocity LinearVel;

    public static ReefscapeCoralOnFly coralOnFly;
    private final IntakeSimulation intakeSimulation;
    
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        
//Use setCustomIntakeCondition(Predicate<GamePieceOnFieldSimulation>)
//  to define a custom condition which controls which game pieces can enter 
//  the intake. Example usage: if you want your simulated intake to only intake 
//  game pieces that are in a specific orentation.



this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
    
    // Specify the type of game pieces that the intake can collect
    "Coral",
    // Specify the drivetrain to which this intake is attached
    driveTrain,
    // Specify width of the intake
    Meters.of(0.7),
    // The intake is mounted on the back side of the chassis
    IntakeSimulation.IntakeSide.FRONT,
    // The intake can hold up to 1 note
    100);

    
         coralOnFly = new ReefscapeCoralOnFly(        // Specify the position of the chassis when the note is launched
        SwerveSub.simPose2dPublic.getTranslation(),
    // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
    new Translation2d(0.2, 0),
    // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
    SwerveSub.simFieldVelocity,
    // The shooter facing direction is the same as the robot’s facing direction
    SwerveSub.simPose2dPublic.getRotation(),
            // Add the shooter’s rotation
    // Initial height of the flying note
    Meters.of(0.45),
    // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
    LinearVel,
    // The angle at which the note is launched
    shooterAngle);//Specify the position of the chassis when the note is launched
        
    coralOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the note will eventually hit the target (if configured)
        (pose3ds) -> Logger.recordOutput("Flywheel/NoteProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        // Callback for when the note will eventually miss the target, or if no target is configured
        (pose3ds) -> Logger.recordOutput("Flywheel/NoteProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
        );
    coralOnFly.addGamePieceAfterTouchGround(null);
    }

     // Defined by IntakeIO
    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    public void launchCorral() {
        // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        if (intakeSimulation.obtainGamePieceFromIntake()) {}
          //  ShooterIOSim.launchNote(); // notify the simulated flywheels to launch a note
    }
    
}


