// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constField;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  Trigger aimAtSpeaker, slowMode, north, south, east, west, chain;
  Measure<Angle> northYaw;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public Drive(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, Trigger slowMode, Trigger north, Trigger east, Trigger south, Trigger west,
      Trigger chain, Trigger aimAtSpeaker) {
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.north = north;
    this.east = east;
    this.south = south;
    this.west = west;
    this.chain = chain;
    this.aimAtSpeaker = aimAtSpeaker;

    isOpenLoop = true;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    northYaw = constField.isRedAlliance() ? Units.Degrees.of(180) : Units.Degrees.of(0);
    redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;
  }

  @Override
  public void execute() {
    if (slowMode.getAsBoolean()) {
      slowMultiplier = prefDrivetrain.slowModeMultiplier.getValue();
    } else {
      slowMultiplier = 1;
    }

    // Get Joystick inputs
    double transMultiplier = slowMultiplier * redAllianceMultiplier
        * Units.Meters.convertFrom(prefDrivetrain.driveSpeed.getValue(), Units.Feet);
    double xVelocity = (xAxis.getAsDouble() * transMultiplier);
    double yVelocity = (-yAxis.getAsDouble() * transMultiplier);

    Measure<Velocity<Angle>> rVelocity = Units.RadiansPerSecond.of(-rotationAxis.getAsDouble())
        .times(prefDrivetrain.turnSpeed.getValue());

    // Requesting snapping ignores any previously calculated rotational speeds
    if (aimAtSpeaker.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker());
    } else if (north.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(northYaw);
    } else if (east.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(northYaw.plus(Units.Degrees.of(270)));
    } else if (south.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(northYaw.plus(Units.Degrees.of(180)));
    } else if (west.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToSnap(northYaw.plus(Units.Degrees.of(90)));
    } else if (chain.getAsBoolean()) {
      rVelocity = subDrivetrain.getVelocityToChain();
    }

    subDrivetrain.drive(new Translation2d(xVelocity, yVelocity), rVelocity.in(Units.RadiansPerSecond), isOpenLoop);
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
