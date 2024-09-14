// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.Autos.WingOnly;
import frc.robot.commands.Zeroing.ZeroClimber;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.commands.Zeroing.ZeroShooterPivot;
import frc.robot.commands.States.Ejecting;
import frc.robot.commands.States.Intaking;
import frc.robot.commands.States.NoneState;
import frc.robot.commands.States.PrepAmp;
import frc.robot.commands.States.PrepShuffle;
import frc.robot.commands.States.PrepSpeaker;
import frc.robot.commands.States.Shooting;
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
  private final SN_XboxController conTestOperator = new SN_XboxController(mapControllers.TEST_OPERATOR_USB);

  private final static StateMachine subStateMachine = new StateMachine();
  private final static Climber subClimber = new Climber();
  private final static Drivetrain subDrivetrain = new Drivetrain();
  private final static Elevator subElevator = new Elevator();
  private final static Intake subIntake = new Intake();
  private final static Shooter subShooter = new Shooter();
  private final static Transfer subTransfer = new Transfer();
  private final static Limelight subLimelight = new Limelight();

  private final Trigger gamePieceTrigger = new Trigger(() -> subTransfer.getGamePieceCollected());

  SendableChooser<Command> autoChooser = new SendableChooser<>();

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
            subElevator, subIntake, subTransfer, subShooter))
            .andThen(Commands.deferredProxy(
                () -> subStateMachine.tryTargetState(subStateMachine, subIntake, subShooter, subTransfer))));

    subDrivetrain.resetModulesToAbsolute();

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureTestBindings(conTestOperator);

    configureAutoSelector();
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
                subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subElevator, subIntake, subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    controller.btn_RightTrigger.whileTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.SHOOTING,
        subStateMachine, subElevator, subIntake, subTransfer, subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subElevator, subIntake, subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    controller.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SPEAKER, subStateMachine, subElevator, subIntake,
                subTransfer, subShooter)));

    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SHUFFLE)))
        .onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_SHUFFLE, subStateMachine,
            subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_A.onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_AMP, subStateMachine,
        subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_B.onTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.STORE_FEEDER,
        subStateMachine, subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_West.whileTrue(Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.EJECTING, subStateMachine, subElevator, subIntake, subTransfer,
            subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subElevator, subIntake, subTransfer,
                subShooter)));
  }

  private void configureTestBindings(SN_XboxController controller) {
    controller.btn_LeftTrigger.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.INTAKING)))
        .whileTrue(new Intaking(subStateMachine, subIntake, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));

    controller.btn_RightTrigger.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.SHOOTING)))
        .whileTrue(new Shooting(subStateMachine, subElevator, subShooter, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));

    controller.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_SPEAKER)))
        .onTrue(new PrepSpeaker(subStateMachine, subShooter));

    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_SHUFFLE)))
        .onTrue(new PrepShuffle(subStateMachine, subShooter));

    controller.btn_A.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_AMP)))
        .onTrue(new PrepAmp(subStateMachine, subElevator, subShooter, subTransfer));

    controller.btn_West.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.EJECTING)))
        .whileTrue(new Ejecting(subStateMachine, subIntake, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("Wing Only Down", new WingOnly(subDrivetrain, true));
    autoChooser.addOption("Wing Only Up", new WingOnly(subDrivetrain, false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
    Command returnedCommand = new ParallelCommandGroup(
        new ZeroClimber(subClimber).withTimeout(constClimber.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroShooterPivot(subShooter).withTimeout(constShooter.ZEROING_TIMEOUT.in(Units.Seconds)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    returnedCommand.addRequirements(subStateMachine);
    return returnedCommand;
  };

}
