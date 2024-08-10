// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AddVisionMeasurement extends Command {
  Drivetrain subDrivetrain;
  Limelight subLimelight;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(Drivetrain subDrivetrain, Limelight subLimelight) {
    this.subDrivetrain = subDrivetrain;
    this.subLimelight = subLimelight;

    addRequirements(subLimelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    LimelightHelpers.SetRobotOrientation("limelight",
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    estimatedPose = subLimelight.getPoseEstimate();
    Measure<Velocity<Angle>> gyroRate = Units.DegreesPerSecond.of(subDrivetrain.getGyroRate());

    if (!subLimelight.rejectUpdate(estimatedPose, gyroRate)) {
      subDrivetrain.addVisionMeasurement(estimatedPose.pose, estimatedPose.timestampSeconds);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
