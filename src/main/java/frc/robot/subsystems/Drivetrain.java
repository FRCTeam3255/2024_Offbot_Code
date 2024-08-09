// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.frcteam3255.components.swerve.SN_SuperSwerve;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constField;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefVision;

public class Drivetrain extends SN_SuperSwerve {
  private static TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
  private static TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
  private static PIDController yawSnappingController;

  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct).publish();
  StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Drivetrain/Desired States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> actualStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/Drivetrain/Actual States", SwerveModuleState.struct).publish();

  private static SN_SwerveModule[] modules = new SN_SwerveModule[] {
      new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
      new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
  };

  public Drivetrain() {
    super(
        constDrivetrain.SWERVE_CONSTANTS,
        modules,
        constDrivetrain.WHEELBASE,
        constDrivetrain.TRACK_WIDTH,
        mapDrivetrain.CAN_BUS_NAME,
        mapDrivetrain.PIGEON_CAN,
        prefDrivetrain.minimumSteerSpeedPercent.getValue(),
        constDrivetrain.DRIVE_MOTOR_INVERT,
        constDrivetrain.STEER_MOTOR_INVERT,
        constDrivetrain.CANCODER_INVERT,
        constDrivetrain.DRIVE_NEUTRAL_MODE,
        constDrivetrain.STEER_NEUTRAL_MODE,
        VecBuilder.fill(
            prefDrivetrain.measurementStdDevsPosition.getValue(),
            prefDrivetrain.measurementStdDevsPosition.getValue(),
            prefDrivetrain.measurementStdDevsHeading.getValue()),
        VecBuilder.fill(
            prefVision.multiTagStdDevsPosition.getValue(),
            prefVision.multiTagStdDevsPosition.getValue(),
            prefVision.multiTagStdDevsHeading.getValue()),
        new PIDConstants(prefDrivetrain.autoDriveP.getValue(),
            prefDrivetrain.autoDriveI.getValue(),
            prefDrivetrain.autoDriveD.getValue()),
        new PIDConstants(prefDrivetrain.autoSteerP.getValue(),
            prefDrivetrain.autoSteerI.getValue(),
            prefDrivetrain.autoSteerD.getValue()),
        new ReplanningConfig(false, true),
        () -> constField.isRedAlliance(),
        Robot.isSimulation());

  }

  @Override
  public void configure() {
    driveConfiguration.Slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.Slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.Slot0.kD = prefDrivetrain.driveD.getValue();

    steerConfiguration.Slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.Slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.Slot0.kD = prefDrivetrain.steerD.getValue();

    SN_SwerveModule.driveConfiguration = driveConfiguration;
    SN_SwerveModule.steerConfiguration = steerConfiguration;

    yawSnappingController = new PIDController(
        prefDrivetrain.yawSnapP.getValue(),
        prefDrivetrain.yawSnapI.getValue(),
        prefDrivetrain.yawSnapD.getValue());
    yawSnappingController.enableContinuousInput(0, 360);

    super.configure();
  }

  public void addEventToAutoMap(String key, Command command) {
    super.autoEventMap.put(key, command);
  }

  /**
   * @param desiredYaw The desired yaw to snap to
   * @return The desired velocity needed to snap.
   */
  public Measure<Velocity<Angle>> getVelocityToSnap(Rotation2d desiredYaw) {
    double yawSetpoint = yawSnappingController.calculate(getRotation().getDegrees(), desiredYaw.getDegrees());

    // limit the PID output to our maximum rotational speed
    yawSetpoint = MathUtil.clamp(yawSetpoint, -prefDrivetrain.turnSpeed.getValue(),
        prefDrivetrain.turnSpeed.getValue());

    return Units.DegreesPerSecond.of(yawSetpoint);
  }

  public Measure<Velocity<Angle>> getVelocityToSnap(Measure<Angle> desiredYaw) {
    return getVelocityToSnap(Rotation2d.fromDegrees(desiredYaw.in(Units.Degrees)));
  }

  /**
   * Calculates the angle necessary for the drivetrain to face a given coordinate.
   * 
   * @param targetPose The coordinate to face
   * @return The necessary angle, in the Field Coordinate System
   */
  public Rotation2d getAngleToTarget(Pose2d targetPose) {
    // Field-relative robot pose
    Pose2d robotPose = getPose();

    // Move the robot pose to be relative to the target
    Pose2d relativeToTarget = robotPose.relativeTo(targetPose);

    // Get the angle of 0,0 to the turret pose
    Rotation2d desiredLockingAngle = new Rotation2d(relativeToTarget.getX(), relativeToTarget.getY());

    // Our shooter is physically mounted 180 degrees from the heading of our
    // drivetrain :o
    desiredLockingAngle = desiredLockingAngle.plus(constShooter.SHOOTER_TO_ROBOT);

    return desiredLockingAngle;
  }

  /**
   * Calculates the angle necessary for the drivetrain to face the speaker.
   * 
   * @return The necessary angle, in the Field Coordinate System
   */
  public Rotation2d getAngleToSpeaker() {
    Pose2d speakerPose = constField.getFieldPositions().get()[0].toPose2d();
    return getAngleToTarget(speakerPose);
  }

  /**
   * Calculates the velocity needed to snap to the nearest chain on your alliance
   * 
   * @return The necessary velocity
   */
  public Measure<Velocity<Angle>> getVelocityToChain() {
    List<Pose2d> chainPositions = constField.getChainPositions();
    Pose2d nearestChain = getPose().nearest(chainPositions);
    return getVelocityToSnap(nearestChain.getRotation());
  }

  /**
   * Resets the driving orientation depending on alliance.
   */
  public void resetDriving(BooleanSupplier isRedAlliance) {
    if (isRedAlliance.getAsBoolean()) {
      resetYaw();
    } else {
      resetYaw(180);
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    for (SN_SwerveModule mod : modules) {
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getDesiredModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Speed (FPS)",
          Units.Meters.convertFrom(Math.abs(getActualModuleStates()[mod.moduleNumber].speedMetersPerSecond),
              Units.Feet));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Desired Angle (Degrees)",
          Math.abs(
              Units.Meters.convertFrom(getDesiredModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Actual Angle (Degrees)",
          Math.abs(Units.Meters.convertFrom(getActualModuleStates()[mod.moduleNumber].angle.getDegrees(), Units.Feet)));

      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Offset Absolute Encoder Angle (Rotations)",
          mod.getAbsoluteEncoder());
      SmartDashboard.putNumber("Drivetrain/Module " + mod.moduleNumber + "/Absolute Encoder Raw Value (Rotations)",
          mod.getRawAbsoluteEncoder());
    }

    robotPosePublisher.set(getPose());
    desiredStatesPublisher.set(getDesiredModuleStates());
    actualStatesPublisher.set(getActualModuleStates());

    SmartDashboard.putNumber("Drivetrain Rotation", getRotation().getDegrees());
  }
}
