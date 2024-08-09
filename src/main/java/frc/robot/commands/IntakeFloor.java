// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotContainer.RobotState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeFloor extends Command {
  Intake subIntake;
  Transfer subTransfer;

  /** Creates a new IntakeFloor. */

  public IntakeFloor(Intake subIntake, Transfer subTransfer) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subIntake, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.setRobotState(RobotState.INTAKING);

    subIntake.setIntakeRollerSpeed(0);
    subTransfer.setFeederSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subIntake.setIntakeRollerSpeed(constIntake.INTAKING_SPEED.in(Units.Percent));
    subTransfer.setFeederSpeed(constTransfer.INTAKING_SPEED.in(Units.Percent));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      RobotContainer.setRobotState(RobotState.STORE_FEEDER);
    } else {
      RobotContainer.setRobotState(RobotState.NONE);
    }

    subIntake.setRollerNeutralOutput();
    subTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.isGamePieceCollected();
  }
}
