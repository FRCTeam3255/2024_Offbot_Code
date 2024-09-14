// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Shooter;

public class ManualPivot extends Command {
  Shooter subShooter;
  DoubleSupplier yAxis;

  /** Creates a new ManualPivot. */
  public ManualPivot(Shooter subShooter, DoubleSupplier yAxis) {
    this.subShooter = subShooter;
    this.yAxis = yAxis;

    addRequirements(subShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.setPivotPercentOutput(Units.Percent.of(yAxis.getAsDouble() * constShooter.MANUAL_PIVOT_PERCENTAGE));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setPivotPosition(subShooter.getShooterPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
