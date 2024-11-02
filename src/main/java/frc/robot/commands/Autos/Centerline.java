// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;

public class Centerline extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  LEDs subLEDs;
  Transfer subTransfer;
  Shooter subShooter;

  BooleanSupplier readyToShoot;
  boolean goesDown;
  private Supplier<Command> shootSequence;

  public Centerline(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake, LEDs subLEDs, Transfer subTransfer, Shooter subShooter, BooleanSupplier readyToShoot,
      boolean goesDown) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subTransfer = subTransfer;
    this.readyToShoot = readyToShoot;
    this.subShooter = subShooter;
    this.goesDown = goesDown;

    shootSequence = () -> new ShootSequence(subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
        subLEDs, subTransfer, subShooter, readyToShoot);

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        // -- PRELOAD --
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber,
            subDrivetrain, subElevator, subIntake, subLEDs, subTransfer, subShooter))
            .until(() -> subTransfer.getGamePieceStored()).withTimeout(3),

        Commands.deferredProxy(shootSequence),

        // -- C5 --
        new PathPlannerAuto(determineInitPathName()),
        Commands.waitSeconds(1),

        // We are now at C5
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".1"),
                Commands.deferredProxy(shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineReturnScorePathName() + ".1")),

            Commands.sequence(
                // It wasnt there :<
                new PathPlannerAuto(determineHopPathName() + ".1")),

            () -> subTransfer.getGamePieceStored()),

        // -- CURRENTLY AT C4 --
        Commands.waitSeconds(1),

        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".2"),
                Commands.deferredProxy(shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineReturnScorePathName() + ".2")),

            Commands.sequence(
                // It wasnt there :<
                new PathPlannerAuto(determineHopPathName() + ".2")),

            () -> subTransfer.getGamePieceStored()),

        // -- CURRENTLY AT C3
        Commands.waitSeconds(1),
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".3"),
                Commands.deferredProxy(shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineReturnScorePathName() + ".3")),

            Commands.sequence(
                // It wasnt there :<
                new PathPlannerAuto(determineHopPathName() + ".3")),

            () -> subTransfer.getGamePieceStored()),

        // -- CURRENTLY AT C2
        Commands.waitSeconds(1),
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".4"),
                Commands.deferredProxy(shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineReturnScorePathName() + ".4")),

            Commands.sequence(
                // It wasnt there :<
                new PathPlannerAuto(determineHopPathName() + ".4")),

            () -> subTransfer.getGamePieceStored()),

        // -- CURRENTLY AT C1
        Commands.waitSeconds(0.5),
        new PathPlannerAuto(determineScorePathName() + ".5"),
        Commands.deferredProxy(shootSequence));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!constField.isRedAlliance())
        ? PathPlannerPath.fromPathFile(determineInitPathName()).getPreviewStartingHolonomicPose()
        : PathPlannerPath.fromPathFile(determineInitPathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  public String determineInitPathName() {
    return "PsC5";
  }

  public String determineScorePathName() {
    return "C5ScoreC1";
  }

  public String determineReturnScorePathName() {
    return "C5ScoreC1R";
  }

  public String determineHopPathName() {
    return "C5HopC1";
  }

}
