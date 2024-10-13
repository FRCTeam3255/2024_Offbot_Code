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

public class Centerline extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  boolean goesDown = false;

  BooleanSupplier readyToShoot = (() -> subDrivetrain.isDrivetrainFacingSpeaker()
      && subShooter.readyToShoot() && subStateMachine.isCurrentStateTargetState()
      && subTransfer.getGamePieceCollected());

  // TODO: Move this into its own command so we can use it everywhere :)
  SequentialCommandGroup shootSequence = new SequentialCommandGroup(
      Commands.waitUntil(() -> subTransfer.getGamePieceCollected()),
      Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

      Commands.parallel(
          Commands.deferredProxy(() -> subStateMachine
              .tryState(RobotState.PREP_VISION, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                  subTransfer,
                  subShooter)
              .repeatedly()),

          Commands.runOnce(() -> subDrivetrain.drive(
              new Translation2d(0, 0),
              subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker()).in(Units.RadiansPerSecond), true))
              .repeatedly())
          .until(readyToShoot),

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

  public Centerline(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake, Transfer subTransfer, Shooter subShooter, boolean goesDown) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        // -- PRELOAD --
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber,
            subDrivetrain, subElevator, subIntake, subTransfer, subShooter))
            .until(() -> subTransfer.getGamePieceCollected()),

        Commands.deferredProxy(() -> shootSequence),

        // -- C5 --
        new PathPlannerAuto(determineInitPathName()),
        Commands.waitSeconds(0.5),

        // We are now at C5
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".1"),
                Commands.deferredProxy(() -> shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".2")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".1"),
            () -> subTransfer.getGamePieceCollected()),

        // -- CURRENTLY AT C4 --
        Commands.waitSeconds(0.5),

        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".3"),
                Commands.deferredProxy(() -> shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".4")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".2"),
            () -> subTransfer.getGamePieceCollected()),

        // -- CURRENTLY AT C3
        Commands.waitSeconds(0.5),

        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".5"),
                Commands.deferredProxy(() -> shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".6")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".3"),
            () -> subTransfer.getGamePieceCollected()),

        // -- CURRENTLY AT C2
        Commands.waitSeconds(0.5),

        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".7"),
                Commands.deferredProxy(() -> shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".8")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".4"),
            () -> subTransfer.getGamePieceCollected()),

        // -- CURRENTLY AT C1
        Commands.waitSeconds(0.5),
        new PathPlannerAuto(determineScorePathName() + ".9"),
        Commands.deferredProxy(() -> shootSequence));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!constField.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determineInitPathName())
        : PathPlannerPath.fromPathFile(determineInitPathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  // TODO: Add goes up functionality
  public String determineInitPathName() {
    return "PsC5";
  }

  public String determineScorePathName() {
    return "C5ScoreC1";
  }

  public String determineHopPathName() {
    return "C5HopC1";
  }

}
