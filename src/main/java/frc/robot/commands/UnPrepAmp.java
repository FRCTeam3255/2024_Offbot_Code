// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constTransfer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
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

    addCommands(
        // Shooter is already pivoted up
        // Run all rollers in reverse until we see a game piece detected
        Commands.runOnce(() -> subTransfer.setFeederSpeed(constTransfer.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subShooter.setShooterPercentOutput(constShooter.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_PREP_TO_AMP_SPEED)),

        Commands.waitUntil(() -> subTransfer.getGamePieceCollected()),

        // continue running all rollers in reverse until we stop seeing that the game
        // piece is detected
        Commands.waitUntil(() -> !subTransfer.getGamePieceCollected())

    // end the command and run the intaking command
    );
  }
}
