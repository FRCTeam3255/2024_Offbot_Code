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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final StateMachine subStateMachine = new StateMachine();
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Intake subIntake = new Intake();
  private final Transfer subTransfer = new Transfer();
  private final Shooter subShooter = new Shooter();
  private final Limelight subLimelight = new Limelight();

  private final Trigger gamePieceTrigger = new Trigger(() -> subTransfer.isGamePieceCollected());

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new Drive(subDrivetrain, subStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper,
                conDriver.btn_Y, conDriver.btn_B, conDriver.btn_A, conDriver.btn_X));

    subLimelight.setDefaultCommand(new AddVisionMeasurement(subDrivetrain, subLimelight));

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);

    gamePieceTrigger
        .onTrue(subStateMachine.tryState(RobotState.STORE_FEEDER, subStateMachine, subIntake, subTransfer, subShooter)
            .andThen(null));

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
        .whileTrue(subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subIntake, subTransfer, subShooter));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public static enum RobotState {
    NONE,
    INTAKING,
    STORE_FEEDER,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_AMP,
    PREP_NONE,
    CLIMBING,
    SHOOTING,
    EJECTING
  }

  public static enum TargetState {
    NONE,
    PREP_SHUFFLE,
    PREP_SPEAKER,
    PREP_AMP,
    PREP_NONE
  }
}
