// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.States;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constField;
import frc.robot.Constants.constStateMachine;
import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;

public class PrepVision extends Command {
  StateMachine subStateMachine;
  Shooter subShooter;
  Drivetrain subDrivetrain;

  Measure<Velocity<Angle>> desiredLeftVelocity, desiredRightVelocity;
  Pose3d[] fieldPoses;
  Pose2d robotPose = new Pose2d();

  public PrepVision(StateMachine subStateMachine, Drivetrain subDrivetrain, Shooter subShooter) {
    this.subStateMachine = subStateMachine;
    this.subShooter = subShooter;
    this.subDrivetrain = subDrivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subStateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState currentRobotState = subStateMachine.getRobotState();

    if (currentRobotState.equals(RobotState.STORE_FEEDER) || subStateMachine.isCurrentStateTargetState()) {
      subStateMachine.setRobotState(RobotState.PREP_VISION);
    }

    fieldPoses = constField.getFieldPositions().get();

    ShooterPositionGroup desiredShooterPosition = constStateMachine.TARGET_TO_PRESET_GROUP.get(TargetState.PREP_VISION);
    subShooter.setDesiredVelocities(desiredShooterPosition.leftVelocity, desiredShooterPosition.rightVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = subDrivetrain.getPose();

    Measure<Angle> calculatedAngle = subShooter.getDesiredAngleToLock(robotPose, fieldPoses);
    subShooter.setPivotPosition(calculatedAngle);

    subShooter.getUpToSpeed();
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
