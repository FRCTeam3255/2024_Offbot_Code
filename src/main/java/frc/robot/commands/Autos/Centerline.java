// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotContainer;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;
import frc.robot.subsystems.Transfer;

public class Centerline extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

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
        Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber,
            subDrivetrain, subElevator, subIntake, subTransfer, subShooter))
            .until(() -> subTransfer.getGamePieceCollected()),

        Commands.waitUntil(() -> subShooter.readyToShoot()),

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

        // -- C5 --
        Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),
        new PathPlannerAuto(determineInitPathName()),
        Commands.waitSeconds(1),

        // We are now at C5
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".1"),
                Commands.waitUntil(() -> subShooter.readyToShoot()),

                // Shoot! (Ends when we don't have a game piece anymore)
                Commands.deferredProxy(() -> subStateMachine
                    .tryState(RobotState.SHOOTING, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                        subTransfer,
                        subShooter)
                    .until(() -> !subTransfer.getGamePieceCollected())),

                // Reset subsystems to chill
                Commands.deferredProxy(() -> subStateMachine
                    .tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                        subTransfer,
                        subShooter)),

                Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".2")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".1"),
            () -> subTransfer.getGamePieceCollected()),

        // -- CURRENTLY AT C4 --
        Commands.waitSeconds(1),

        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".2"),
                Commands.waitUntil(() -> subShooter.readyToShoot()),

                // Shoot! (Ends when we don't have a game piece anymore)
                Commands.deferredProxy(() -> subStateMachine
                    .tryState(RobotState.SHOOTING, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                        subTransfer,
                        subShooter)
                    .until(() -> !subTransfer.getGamePieceCollected())),

                // Reset subsystems to chill
                Commands.deferredProxy(() -> subStateMachine
                    .tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                        subTransfer,
                        subShooter)),

                Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

                // Return to centerline (C4)
                new PathPlannerAuto(determineScorePathName() + ".3")),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + ".2"),
            () -> subTransfer.getGamePieceCollected())

    // -- CURRENTLY AT C3
    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!constField.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determineInitPathName())
        : PathPlannerPath.fromPathFile(determineInitPathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  public String determineInitPathName() {
    return "PsC5";
  }

  public String determineScorePathName() {
    return "C5ScoreC1";
  }

  public String determineHopPathName() {
    return "C5HopToC1";
  }

}
