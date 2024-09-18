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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;

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

    addCommands(
        Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_AMP)),

        // Set amp position of elevator and wait until it's at position
        Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.AMP_POSITION)),
        Commands.waitUntil(() -> subElevator.isElevatorAtPosition(constElevator.AMP_POSITION)),

        // Set amp position of shooter and wait until it's at position
        Commands.runOnce(() -> subShooter.setPivotPosition(constShooter.TRANSFER_TO_AMPER_ANGLE)),
        Commands.waitUntil(() -> subShooter.isShooterAtPosition(constShooter.TRANSFER_TO_AMPER_ANGLE)),

        // Spin feeder, shooter, and drainpipe motors
        Commands.runOnce(() -> subTransfer.setFeederSpeed(constTransfer.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subShooter.setShooterPercentOutput(constShooter.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_PREP_TO_AMP_SPEED)),

        // Wait for the note to transfer to drainpipe
        Commands.waitSeconds(constElevator.PREP_AMP_DELAY.in(Units.Seconds)),

        // Stop motors
        Commands.runOnce(() -> subTransfer.setFeederSpeed(constTransfer.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subShooter.setShooterPercentOutput(constShooter.PREP_TO_AMP_SPEED)),
        Commands.runOnce(() -> subElevator.setDrainpipeSpeed(constElevator.DRAINPIPE_PREP_TO_AMP_SPEED)));
  }
}
