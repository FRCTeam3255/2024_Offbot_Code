// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constClimber;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.Zeroing.ZeroClimber;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.commands.Zeroing.ZeroShooterPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final static StateMachine subStateMachine = new StateMachine();
  private final static Climber subClimber = new Climber();
  private final static Drivetrain subDrivetrain = new Drivetrain();
  private final static Elevator subElevator = new Elevator();
  private final static Intake subIntake = new Intake();
  private final static Transfer subTransfer = new Transfer();
  private final static Shooter subShooter = new Shooter();
  private final static Limelight subLimelight = new Limelight();

  private final Trigger gamePieceTrigger = new Trigger(() -> subTransfer.getGamePieceCollected());

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new Drive(subDrivetrain, subStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper,
                conDriver.btn_Y, conDriver.btn_B, conDriver.btn_A, conDriver.btn_X));

    // subLimelight.setDefaultCommand(new AddVisionMeasurement(subDrivetrain,
    // subLimelight));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    gamePieceTrigger
        .onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.STORE_FEEDER, subStateMachine,
            subElevator, subIntake, subTransfer,
            subShooter)));

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_Back.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[6].toPose2d())));

    // Defaults to Field-Relative, is Robot-Relative while held
    controller.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));

  }

  private void configureOperatorBindings(SN_XboxController controller) {
    controller.btn_LeftTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subElevator, subIntake, subTransfer,
                subShooter)));

    controller.btn_RightTrigger.whileTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.SHOOTING,
        subStateMachine, subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SPEAKER, subStateMachine, subElevator, subIntake,
                subTransfer, subShooter)));

    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SHUFFLE)))
        .onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_SHUFFLE, subStateMachine,
            subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_A.onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_AMP, subStateMachine,
        subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_West.whileTrue(Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.EJECTING, subStateMachine, subElevator, subIntake, subTransfer,
            subShooter)));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Returns the command to zero all subsystems. This will make all subsystems
   * move
   * themselves downwards until they see a current spike and cancel any incoming
   * commands that
   * require those motors. If the zeroing does not end within a certain time
   * frame (set in constants), it will interrupt itself.
   * 
   * @return Parallel commands to zero the Climber, Elevator, and Shooter Pivot
   */
  public static Command zeroSubsystems() {
    return new ParallelCommandGroup(
        new ZeroClimber(subClimber).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .withTimeout(constClimber.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroElevator(subElevator).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroShooterPivot(subShooter).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .withTimeout(constShooter.ZEROING_TIMEOUT.in(Units.Seconds)));
  }
}
