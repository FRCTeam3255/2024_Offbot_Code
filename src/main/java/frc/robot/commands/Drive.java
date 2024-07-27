// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  Trigger aimAtSpeaker;

  public Drive(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, Trigger aimAtSpeaker) {
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.aimAtSpeaker = aimAtSpeaker;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get Joystick inputs
    double xVelocity = xAxis.getAsDouble() * Units.Meters.convertFrom(prefDrivetrain.driveSpeed.getValue(), Units.Feet);
    double yVelocity = -yAxis.getAsDouble()
        * Units.Meters.convertFrom(prefDrivetrain.driveSpeed.getValue(), Units.Feet);
    double rVelocity = -rotationAxis.getAsDouble()
        * Units.Radians.convertFrom(prefDrivetrain.turnSpeed.getValue(), Units.Degrees);

    // Previous codebase checks if the driver is rotating at all before doing
    // this...
    // TODO: Ask Kevin about his preference
    // Snapping ignores previouly calculated rotational speeds
    if (aimAtSpeaker.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker()).in(Units.RadiansPerSecond);
    }

    subDrivetrain.drive(new Translation2d(xVelocity, yVelocity), rVelocity, isOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
