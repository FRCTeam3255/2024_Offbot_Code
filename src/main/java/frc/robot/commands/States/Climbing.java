// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public class Climbing extends SequentialCommandGroup {

  ShooterPositionGroup desiredShooterPosition = constShooter.CLIMBING;

  public Climbing(Climber subClimber, Elevator subElevator, StateMachine subStateMachine, Shooter subShooter,
      Transfer subTransfer) {
    addRequirements(subStateMachine);

    addCommands(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.CLIMBING)),
        Commands.runOnce(
            () -> subShooter.setDesiredVelocities(Units.RotationsPerSecond.of(-30), Units.RotationsPerSecond.of(-30))),
        Commands.runOnce(() -> subShooter.getUpToSpeed()),

        // Check if we have a gp: if we do, put it in the drainpipe using prep amp
        Commands.either(
            Commands.sequence(
                new PrepTargetState(subElevator, subStateMachine, subShooter,
                    TargetState.PREP_AMP),
                // Spin feeder and drainpipe motors
                Commands.runOnce(() -> subTransfer.setFeederSpeed(constTransfer.PREP_TO_AMP_SPEED)),
                Commands.runOnce(() -> subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_PREP_TO_AMP_SPEED)),
                // Wait for the note to transfer to drainpipe
                Commands.waitUntil(() -> subElevator.getGamePieceStored()),
                // Stop motors
                Commands.runOnce(() -> subTransfer.setFeederSpeed(0)),
                Commands.runOnce(() -> subElevator.setDrainpipeSpeed(0))),

            Commands.print("Climb was initiated, but no Game Piece was found!"),
            () -> subTransfer.getGamePieceCollected()),

        // Move the elevator and shooter to a point where we can safely climb
        Commands.runOnce(() -> subElevator.setElevatorPosition(desiredShooterPosition.elevatorPosition)),
        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(desiredShooterPosition.elevatorPosition)),
        Commands.runOnce(() -> subShooter.setDesiredPosition(desiredShooterPosition)),
        Commands.waitUntil(() -> subShooter.isShooterAtPosition(desiredShooterPosition.shooterAngle)),

        // Driver will now use joysticks to move the climbers up and down
        Commands.runOnce(() -> subClimber.setSafeToMoveClimber(true)),

        Commands.waitUntil(() -> subClimber.getClimberPosition().in(Units.Meters) >= 0.3),
        Commands.runOnce(() -> subShooter.setShootingNeutralOutput()),
        Commands.runOnce(() -> subShooter.setDesiredVelocities(Units.RotationsPerSecond.zero(),
            Units.RotationsPerSecond.zero()))

    );
  }
}
