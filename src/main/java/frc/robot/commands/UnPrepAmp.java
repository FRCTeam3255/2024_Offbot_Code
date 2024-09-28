// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
        // Shooter is currently pivoted up and the elevator is up (bad news bears!)
        // Stop those rollers
        Commands.runOnce(() -> subShooter.setShootingNeutralOutput()),
        Commands.runOnce(
            () -> subShooter.setDesiredVelocities(Units.RotationsPerSecond.zero(), Units.RotationsPerSecond.zero())),

        // Pivot shooter back
        Commands.runOnce(() -> subShooter.setPivotPosition(constShooter.PIVOT_BACKWARD_INTAKE_LIMIT)),
        Commands.waitUntil(() -> subShooter.getShooterPosition().lte(constShooter.ELEVATOR_ABLE_TO_MOVE_LIMIT)),

        // Lower the elevator when it's safe
        Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT))
    // End this command and run the intaking command
    );
  }
}
