// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constLimelight;

public class Limelight extends SubsystemBase {
  PoseEstimate lastPoseEstimate = new PoseEstimate();

  public Limelight() {
  }

  public PoseEstimate getPoseEstimate() {
    return lastPoseEstimate;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro. <b>
   *                     Units: </b> Rotations per second
   * @return True if the estimate should be rejected
   */
  public boolean rejectUpdate(PoseEstimate poseEstimate, Measure<Velocity<Angle>> gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(constLimelight.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :[
    if (poseEstimate.tagCount == 0) {
      return true;
    }
    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && LimelightHelpers.getTA("limelight") > constLimelight.AREA_THRESHOLD) {
      return false;
      // 2 tags
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    PoseEstimate currentEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (currentEstimate != null) {
      lastPoseEstimate = currentEstimate;
    }
  }
}
