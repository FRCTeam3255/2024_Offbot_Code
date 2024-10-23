// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;
import frc.robot.subsystems.Transfer;

public class ShootSequence extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;

  BooleanSupplier readyToShoot;

  public ShootSequence(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake, Transfer subTransfer, Shooter subShooter, BooleanSupplier readyToShoot) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.readyToShoot = readyToShoot;

    addCommands(
        new SequentialCommandGroup(
            Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)),

            Commands.parallel(
                Commands.deferredProxy(() -> subStateMachine
                    .tryState(RobotState.PREP_VISION, subStateMachine, subClimber, subDrivetrain, subElevator,
                        subIntake,
                        subTransfer,
                        subShooter)
                    .repeatedly()),

                Commands.runOnce(() -> subDrivetrain.drive(
                    new Translation2d(0, 0),
                    subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker()).in(Units.RadiansPerSecond),
                    true))
                    .repeatedly())
                .until(readyToShoot),

            // Shoot! (Ends when we don't have a game piece anymore)
            Commands.deferredProxy(() -> subStateMachine
                .tryState(RobotState.SHOOTING, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                    subTransfer,
                    subShooter)
                .until(() -> !subTransfer.getGamePieceStored())),

            // Reset subsystems to chill
            Commands.deferredProxy(() -> subStateMachine
                .tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                    subTransfer,
                    subShooter)),

            Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)))
            .unless(() -> !subTransfer.getGamePieceStored()));

  }
}
