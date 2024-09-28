// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constShooter;
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

        // Intake
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter)),

        // Drive to first note
        new PathPlannerAuto("PsW3sW2sW1s.1"),

        // Aim at speaker
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_VISION, subStateMachine, subDrivetrain, subElevator,
                subIntake, subTransfer, subShooter)),

        // Shoot
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.SHOOTING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter).until(() -> !subTransfer.getGamePieceCollected())),

        // Intake
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter)),

        // Drive to second note
        new PathPlannerAuto("PsW3sW2sW1s.2"),

        // Aim at speaker
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_VISION, subStateMachine, subDrivetrain, subElevator,
                subIntake, subTransfer, subShooter)),

        // Shoot
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.SHOOTING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter).until(() -> !subTransfer.getGamePieceCollected())),

        // Intake
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter)),

        // Drive to third note
        new PathPlannerAuto("PsW3sW2sW1s.3"),

        // Aim at speaker
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_VISION, subStateMachine, subDrivetrain, subElevator,
                subIntake, subTransfer, subShooter)),

        // Shoot
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.SHOOTING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer, subShooter).until(() -> !subTransfer.getGamePieceCollected())),

        Commands.waitSeconds(constShooter.AUTO_PREP_NONE_DELAY.in(Units.Seconds)),

        // Reset subsystems to chill
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subDrivetrain,
            subElevator, subIntake, subTransfer, subShooter)));
  }

  public String determinePathName() {
    return (goesDown) ? "PsW1sW2sW3s" : "PsW3sW2sW1s";
  }

  public Supplier<Pose2d> getInitialPose() {
    // only for blue alliance at the moment
    return () -> PathPlannerAuto.getStaringPoseFromAutoFile(determinePathName());
  }
}
