// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.States.Ejecting;
import frc.robot.commands.States.Intaking;
import frc.robot.commands.States.PrepShuffle;
import frc.robot.commands.States.PrepSpeaker;
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
  private final SN_XboxController conTestOperator = new SN_XboxController(mapControllers.TEST_OPERATOR_USB);

  private final StateMachine subStateMachine = new StateMachine();
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Elevator subElevator = new Elevator();
  private final Intake subIntake = new Intake();
  private final Transfer subTransfer = new Transfer();
  private final Shooter subShooter = new Shooter();
  private final Limelight subLimelight = new Limelight();

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

    gamePieceTrigger
        .onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.STORE_FEEDER, subStateMachine,
            subElevator, subIntake, subTransfer,
            subShooter)));

    subDrivetrain.resetModulesToAbsolute();

    configureDriverBindings();
    configureOperatorBindings();
    configureTestBindings();
  }

  public void configureDriverBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Back.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[6].toPose2d())));

    // Defaults to Field-Relative, is Robot-Relative while held
    conDriver.btn_LeftBumper
        .whileTrue(Commands.runOnce(() -> subDrivetrain.setRobotRelative()))
        .onFalse(Commands.runOnce(() -> subDrivetrain.setFieldRelative()));
  }

  public void configureOperatorBindings() {
    conOperator.btn_LeftTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subElevator, subIntake, subTransfer,
                subShooter)));

    conOperator.btn_RightTrigger.whileTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.SHOOTING,
        subStateMachine, subElevator, subIntake, subTransfer, subShooter)));

    conOperator.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SPEAKER, subStateMachine, subElevator, subIntake,
                subTransfer, subShooter)));

    conOperator.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SHUFFLE)))
        .onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_SHUFFLE, subStateMachine,
            subElevator, subIntake, subTransfer, subShooter)));

    conOperator.btn_A.onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_AMP, subStateMachine,
        subElevator, subIntake, subTransfer, subShooter)));

    conOperator.btn_West.whileTrue(Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.EJECTING, subStateMachine, subElevator, subIntake, subTransfer,
            subShooter)));
  }

  public void configureTestBindings() {
    conTestOperator.btn_LeftTrigger.whileTrue(new Intaking(subStateMachine, subIntake, subTransfer));
    conTestOperator.btn_Y.onTrue(new PrepSpeaker(subStateMachine, subShooter));
    conTestOperator.btn_X.onTrue(new PrepShuffle(subStateMachine, subShooter));
    conTestOperator.btn_West.whileTrue(new Ejecting(subStateMachine, subIntake, subTransfer));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
