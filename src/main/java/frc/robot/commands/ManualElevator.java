// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constElevator;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
  Elevator subElevator;
  DoubleSupplier yAxis;

  /** Creates a new ManualPivot. */
  public ManualElevator(Elevator subElevator, DoubleSupplier yAxis) {
    this.subElevator = subElevator;
    this.yAxis = yAxis;

    addRequirements(subElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subElevator.setElevatorSpeed(yAxis.getAsDouble() * constElevator.MANUAL_ELEVATOR_PERCENTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subElevator.setElevatorPosition(subElevator.getElevatorPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
