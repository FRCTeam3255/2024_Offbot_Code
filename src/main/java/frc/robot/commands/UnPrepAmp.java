// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constStateMachine;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.TargetState;
import frc.robot.subsystems.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnPrepAmp extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Elevator subElevator;
  Shooter subShooter;
  Transfer subTransfer;

  public UnPrepAmp(StateMachine subStateMachine, Elevator subElevator, Shooter subShooter, Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subElevator = subElevator;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    addRequirements(subStateMachine);

    ShooterPositionGroup desiredShooterPosition = constStateMachine.TARGET_TO_PRESET_GROUP.get(TargetState.PREP_AMP);

    addCommands(
        // Shooter is currently pivoted up and the elevator is up (bad news bears!)
        // Lets double check that though while also setting the flywheels in reverse
        Commands
            .runOnce(() -> subShooter.setDesiredPosition(new ShooterPositionGroup(desiredShooterPosition.shooterAngle,
                desiredShooterPosition.leftVelocity.negate(), desiredShooterPosition.rightVelocity.negate()))),

        // Feed the note backwards until we see the note
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(-constElevator.DRAINPIPE_SCORE_AMP_SPEED)),
        Commands.runOnce(() -> subTransfer.setFeederSpeed(-constTransfer.PREP_TO_AMP_SPEED)),

        Commands.waitUntil(() -> subTransfer.getGamePieceCollected()),
        // Continue feeding until we stop seeing it
        Commands.waitUntil(() -> !subTransfer.getGamePieceCollected()),

        // Stop all rollers
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(0)),
        Commands.runOnce(() -> subTransfer.setFeederSpeed(0)),
        Commands
            .runOnce(() -> subShooter.setDesiredVelocities(Units.DegreesPerSecond.of(0), Units.DegreesPerSecond.of(0))),
        Commands.runOnce(() -> subShooter.setShootingNeutralOutput()),

        // Pivot shooter back
        Commands.runOnce(() -> subShooter.setPivotPosition(constShooter.PIVOT_BACKWARD_INTAKE_LIMIT)),
        Commands.waitUntil(() -> subShooter.getShooterPosition().lte(constShooter.ELEVATOR_ABLE_TO_MOVE_LIMIT)),

        // Lower the elevator when it's safe
        Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT))
    // End this command and run the intaking command
    );
  }
}
