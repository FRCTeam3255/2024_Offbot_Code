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

        // Pivot shooter and then start spinning the flywheels :p
        Commands.runOnce(() -> subShooter.setDesiredPosition(desiredShooterPosition)));
  }
}
