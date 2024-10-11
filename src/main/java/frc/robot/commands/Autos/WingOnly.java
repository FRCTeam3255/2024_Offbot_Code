// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WingOnly extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  boolean goesDown;

  BooleanSupplier readyToShoot = (() -> subDrivetrain.isDrivetrainFacingSpeaker()
      && subShooter.readyToShoot() && subStateMachine.isCurrentStateTargetState()
      && subTransfer.getGamePieceCollected());

  // TODO: Move this into its own command so we can use it everywhere :)
  SequentialCommandGroup shootSequence = new SequentialCommandGroup(
      Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

      Commands.runOnce(() -> subDrivetrain.drive(
          new Translation2d(0, 0),
          subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker()).in(Units.RadiansPerSecond), true))
          .repeatedly().until(readyToShoot),

      // Shoot! (Ends when we don't have a game piece anymore)
      Commands.deferredProxy(() -> subStateMachine
          .tryState(RobotState.SHOOTING, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
              subTransfer,
              subShooter)
          .until(() -> !subTransfer.getGamePieceCollected())),

      // Reset subsystems to chill
      Commands.deferredProxy(() -> subStateMachine
          .tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake, subTransfer,
              subShooter)),

      Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)));

  /** Creates a new WingDown. */
  public WingOnly(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake, Transfer subTransfer, Shooter subShooter, boolean goesDown) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;

    this.goesDown = goesDown;

    addCommands(
        // Resetting pose
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(
            getInitialPose().get())),

        // -- PRELOAD --
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber,
            subDrivetrain, subElevator, subIntake, subTransfer, subShooter))
            .until(() -> subTransfer.getGamePieceCollected()),

        Commands.deferredProxy(() -> shootSequence),

        // -- W1 / W3 --
        // Drive to first note (Intaking is within the path)
        new PathPlannerAuto(determinePathName() + ".1"),

        Commands.waitUntil(readyToShoot),
        Commands.deferredProxy(() -> shootSequence),

        // -- W2 --
        // Drive to first note (Intaking is within the path)
        new PathPlannerAuto(determinePathName() + ".2"),

        Commands.waitUntil(readyToShoot),
        Commands.deferredProxy(() -> shootSequence),

        // -- W3 / W1 --
        // Drive to first note (Intaking is within the path)
        new PathPlannerAuto(determinePathName() + ".3"),

        Commands.waitUntil(readyToShoot),
        Commands.deferredProxy(() -> shootSequence),

        // Reset subsystems to chill
        Commands.deferredProxy(() -> subStateMachine
            .tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain,
                subElevator, subIntake, subTransfer,
                subShooter)));
  }

  public String determinePathName() {
    return (goesDown) ? "PsW1W2W3" : "PsW3W2W1";
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!constField.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determinePathName())
        : PathPlannerPath.fromPathFile(determinePathName()).flipPath().getPreviewStartingHolonomicPose();
  }
}
