// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constShooter;
import frc.robot.commands.States.NoneState;
import frc.robot.commands.States.StoreFeeder;
import frc.robot.subsystems.Climber;
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
public class PreloadTaxi extends SequentialCommandGroup {
  StateMachine subStateMachine;
  Climber subClimber;
  Drivetrain subDrivetrain;
  Elevator subElevator;
  Intake subIntake;
  Shooter subShooter;
  Transfer subTransfer;

  /** Creates a new PreloadTaxi. */
  public PreloadTaxi(StateMachine subStateMachine, Climber subClimber, Drivetrain subDrivetrain, Elevator subElevator,
      Intake subIntake,
      Shooter subShooter,
      Transfer subTransfer) {
    this.subStateMachine = subStateMachine;
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;
    this.subElevator = subElevator;
    this.subIntake = subIntake;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Resetting pose
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(
            getInitialPose().get())),

        // Skip directly to STORE_FEEDER since we already have a game piece
        new StoreFeeder(subStateMachine, subIntake, subTransfer, subShooter),

        // Aim at Speaker TODO: maybe make presets later?
        Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_VISION, subStateMachine, subClimber, subDrivetrain,
                subElevator,
                subIntake,
                subTransfer, subShooter)),

        // Shoot! (Ends when we don't have a game piece anymore)
        Commands.deferredProxy(() -> subStateMachine
            .tryState(RobotState.SHOOTING, subStateMachine, subClimber, subDrivetrain, subElevator, subIntake,
                subTransfer,
                subShooter)
            .until(() -> !subTransfer.getGamePieceCollected())),

        Commands.waitSeconds(constShooter.AUTO_PREP_NONE_DELAY.in(Units.Seconds)),

        // Reset subsystems to chill
        Commands
            .deferredProxy(() -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain,
                subElevator, subIntake, subTransfer,
                subShooter)),

        // Mooovve outside starting line
        new PathPlannerAuto("PsTaxi"));
  }

  public Supplier<Pose2d> getInitialPose() {
    // only for blue alliance at the moment
    return () -> PathPlannerAuto.getStaringPoseFromAutoFile("PsTaxi");
  }
}
