// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeFloor extends Command {
  Intake subIntake;
  Transfer subTransfer;

  /** Creates a new IntakeFloor. */

  public IntakeFloor(Intake subIntake, Transfer subTransfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    addRequirements(subIntake, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.currentState = RobotContainer.robotState.INTAKING;
    this.subIntake.setIntakeRollerSpeed(0);
    this.subTransfer.setFeederSpeed(0);
    // if subTransfer.DigitalInput();
    // this.subIntake.setRollerNeutralOutput();
    // this.subTransfer.setFeederNeutralOutput();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
