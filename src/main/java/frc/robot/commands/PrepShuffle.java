// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class PrepShuffle extends Command {
  Drivetrain subDrivetrain;
  Shooter subShooter;

  boolean canShuffle = false;

  /** Creates a new PrepShuffle. */
  public PrepShuffle(Shooter subShooter, Drivetrain subDrivetrain) {
    this.subShooter = subShooter;
    this.subDrivetrain = subDrivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.getRobotState() == RobotState.STORE_FEEDER) {
      subShooter.setShooterPosition(null);
      // TODO: NULL POINTER EXCEPTION MY BELOVED!
      // This will be an interpolating tree map in Constants one day
      canShuffle = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canShuffle) {
      RobotContainer.setRobotState(RobotState.PREP_SHUFFLE);
    }
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
