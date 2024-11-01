// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.nio.file.Paths;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
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
import frc.robot.subsystems.StateMachine.TargetState;

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

  public SequentialCommandGroup getScoreOrHopCmd(int noteNumber) {
    String pathSuffix = "." + noteNumber;

    return new SequentialCommandGroup(
        Commands.waitSeconds(0.5),

        // We are now at C5
        Commands.either(
            Commands.sequence(
                // We got the game piece!
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + pathSuffix),
                Commands.deferredProxy(shootSequence),
                // Return to centerline (C4)
                new PathPlannerAuto(determineReturnScorePathName() + pathSuffix)),

            // It wasnt there :<
            new PathPlannerAuto(determineHopPathName() + pathSuffix),
            () -> subTransfer.getGamePieceStored()));
  }

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
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING))
            .until(() -> subTransfer.getGamePieceStored()),

        Commands.deferredProxy(shootSequence),

        // -- C5 --
        new PathPlannerAuto(determineInitPathName()),
        Commands.deferredProxy(() -> getScoreOrHopCmd(1)),

        // -- CURRENTLY AT C4 --
        Commands.deferredProxy(() -> getScoreOrHopCmd(2)),

        // -- CURRENTLY AT C3
        Commands.deferredProxy(() -> getScoreOrHopCmd(3)),

        // -- CURRENTLY AT C2
        Commands.deferredProxy(() -> getScoreOrHopCmd(4)),

        // -- CURRENTLY AT C1
        Commands.waitSeconds(0.5),
        new PathPlannerAuto(determineScorePathName() + ".5"),
        Commands.deferredProxy(shootSequence));
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

  public String determineReturnScorePathName() {
    return "C5ScoreC1R";
  }

  public String determineHopPathName() {
    return "C5HopC1";
  }

}
