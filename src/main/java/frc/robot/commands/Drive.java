// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constField;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.StateMachine;

public class Drive extends Command {
  Drivetrain subDrivetrain;
  StateMachine subStateMachine;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;
  Trigger slowMode, north, south, east, west, chain;
  Measure<Angle> northYaw;
  double redAllianceMultiplier = 1;
  double slowMultiplier = 0;

  public Drive(Drivetrain subDrivetrain, StateMachine subStateMachine, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, Trigger slowMode, Trigger chain, Trigger north, Trigger east, Trigger south,
      Trigger west) {
    this.subDrivetrain = subDrivetrain;
    this.subStateMachine = subStateMachine;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.north = north;
    this.east = east;
    this.south = south;
    this.west = west;
    this.chain = chain;

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
        * prefDrivetrain.driveSpeed.getValue();

    Measure<Velocity<Distance>> xVelocity = Units.MetersPerSecond.of(xAxis.getAsDouble() * transMultiplier);
    Measure<Velocity<Distance>> yVelocity = Units.MetersPerSecond.of(-yAxis.getAsDouble() * transMultiplier);

    Measure<Velocity<Angle>> rVelocity = Units.RadiansPerSecond
        .of(prefDrivetrain.maxManualTurnSpeed.in(Units.DegreesPerSecond))
        .times(-rotationAxis.getAsDouble());

    // Requesting snapping ignores any previously calculated rotational speeds
    if (north.getAsBoolean()) {
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

    // Ignore calculated rotation if a driver rotation is given
    if (rVelocity.equals(Units.RadiansPerSecond.of(0))) {
      switch (subStateMachine.getRobotState()) {
        case PREP_SHUFFLE:
          rVelocity = subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToShuffle());
          break;
        // case PREP_AMP:
        // rVelocity = subDrivetrain.getVelocityToSnap(Units.Degrees.of(90));
        // break;
        case PREP_VISION:
          rVelocity = subDrivetrain.getVelocityToSnap(subDrivetrain.getAngleToSpeaker());
          break;
        default:
          break;
      }

    }

    subDrivetrain.drive(new Translation2d(xVelocity.in(Units.MetersPerSecond), yVelocity.in(Units.MetersPerSecond)),
        rVelocity.in(Units.RadiansPerSecond), isOpenLoop);
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
