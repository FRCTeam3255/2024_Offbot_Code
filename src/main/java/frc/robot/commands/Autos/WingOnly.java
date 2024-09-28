// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WingOnly extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;
  boolean goesDown;

  /** Creates a new WingDown. */
  public WingOnly(StateMachine subStateMachine, Drivetrain subDrivetrain, Elevator subElevator, Intake subIntake,
      boolean goesDown) {
    this.subStateMachine = subStateMachine;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.goesDown = goesDown;

    addCommands(
        // Resetting pose
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            getInitialPose().get().getRotation().getDegrees())),
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        // new PathPlannerAuto("PsW1sW2sW3s"),

        // Intake
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter)),

        // Drive to first note
        new PathPlannerAuto(determinePathName() + ".1")

        // Transfer + Shoot

        // Intake

        // Drive to second note
        // new PathPlannerAuto(determinePathName() + ".2"),

        // Transfer + Shoot

        // Intake

        // Drive to third note
        // new PathPlannerAuto(determinePathName() + ".3")

        // Transfer + Shoot
    );
  }

  public String determinePathName() {
    return (goesDown) ? "PsW1sW2sW3s" : "PsW3sW2sW1s";
  }

  public Supplier<Pose2d> getInitialPose() {
    // only for blue alliance at the moment
    return () -> PathPlannerAuto.getStaringPoseFromAutoFile(determinePathName());
  }
}
