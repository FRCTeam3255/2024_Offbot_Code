// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import java.lang.annotation.Target;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constStateMachine;
import frc.robot.Constants.constTransfer;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepAmp extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Elevator subElevator;
  Shooter subShooter;
  Transfer subTransfer;

  /** Creates a new PrepAmp. */
  public PrepAmp(StateMachine subStateMachine, Elevator subElevator, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subElevator = subElevator;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    addRequirements(subStateMachine);

    ShooterPositionGroup desiredShooterPosition = constStateMachine.TARGET_TO_PRESET_GROUP.get(TargetState.PREP_AMP);

    // Implementation of this command assumes that the current robot state is either
    // STORE_FEEDER or a target state.
    addCommands(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_AMP)),

        // Set amp position of elevator and wait until it's at position
        Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.AMP_POSITION)),
        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(constElevator.AMP_POSITION)),

        Commands.waitSeconds(2),

        // Pivot shooter
        Commands.runOnce(() -> subShooter.setDesiredPosition(desiredShooterPosition)),
        Commands.waitUntil(() -> subShooter.isShooterAtPosition(desiredShooterPosition.shooterAngle)),

        Commands.waitSeconds(2),

        // Feed the note into the drainpipe until we see a note in that bad boy
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_SCORE_AMP_SPEED)),
        Commands.runOnce(() -> subTransfer.setFeederSpeed(constTransfer.PREP_TO_AMP_SPEED)),

        // TODO: ADD A TIMEOUT HERE
        Commands.waitUntil(() -> subElevator.getGamePieceStored()),

        // then turn everything off :>
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(0)),
        Commands.runOnce(() -> subTransfer.setFeederSpeed(0)),
        Commands
            .runOnce(() -> subShooter.setDesiredVelocities(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))),
        Commands.runOnce(() -> subShooter.setShootingNeutralOutput())

    );
  }
}
