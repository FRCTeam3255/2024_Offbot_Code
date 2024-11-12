// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreloadOnly extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  LEDs subLEDs;
  Shooter subShooter;
  Transfer subTransfer;
  int position = 0;
  DoubleSupplier secondSupplier;

  // BLUE
  Pose2d SubAmpSideBlue = new Pose2d(0.602, 6.747, Rotation2d.fromDegrees(60));
  Pose2d SubCenterBlue = new Pose2d(1.360, 5.563, Rotation2d.fromDegrees(0));
  Pose2d SubSourceSideBlue = new Pose2d(0.602, 4.348, Rotation2d.fromDegrees(-60));
  Pose2d[] startingPositionsBlue = { SubAmpSideBlue, SubCenterBlue, SubSourceSideBlue };

  // RED
  Pose2d SubAmpSideRed = new Pose2d(constField.FIELD_LENGTH.in(Units.Meters) - 0.602, 6.747,
      Rotation2d.fromDegrees(120));
  Pose2d SubCenterRed = new Pose2d(constField.FIELD_LENGTH.in(Units.Meters) - 1.360, 5.563,
      Rotation2d.fromDegrees(180));
  Pose2d SubSourceSideRed = new Pose2d(constField.FIELD_LENGTH.in(Units.Meters) - 0.602, 4.348,
      Rotation2d.fromDegrees(-120));
  Pose2d[] startingPositionsRed = { SubAmpSideRed, SubCenterRed, SubSourceSideRed };

  /** Creates a new PreloadOnly. */
  public PreloadOnly(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake, LEDs subLEDs, Shooter subShooter, Transfer subTransfer, int position,
      DoubleSupplier secondSupplier) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.position = position;
    this.secondSupplier = secondSupplier;

    addCommands(
        // Resetting pose
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(
            getInitialPose().get())),

        Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)),

        Commands.waitSeconds(secondSupplier.getAsDouble()),

        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKING))
            .until(() -> subTransfer.getGamePieceStored()),

        Commands.waitUntil(() -> subShooter.readyToShoot()),

        // Shoot! (Ends when we don't have a game piece anymore)
        Commands.deferredProxy(() -> subStateMachine
            .tryState(RobotState.SHOOTING)
            .until(() -> !subTransfer.getGamePieceStored())),

        // Reset subsystems to chill
        Commands.deferredProxy(() -> subStateMachine
            .tryState(RobotState.NONE)));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (constField.isRedAlliance())
        ? startingPositionsRed[position]
        : startingPositionsBlue[position];
  }

}
