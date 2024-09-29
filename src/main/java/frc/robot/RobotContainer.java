// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.ManualElevator;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.Zeroing.ZeroClimber;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.commands.Zeroing.ZeroShooterPivot;
import frc.robot.commands.States.Ejecting;
import frc.robot.commands.States.Intaking;
import frc.robot.commands.States.NoneState;
import frc.robot.commands.States.PrepTargetState;
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
  private final Trigger readyToShoot = new Trigger(() -> subDrivetrain.isDrivetrainFacingSpeaker());

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
        .onTrue(Commands
            .deferredProxy(() -> subStateMachine.tryState(RobotState.STORE_FEEDER, subStateMachine, subDrivetrain,
                subElevator, subIntake, subTransfer, subShooter))
            .andThen(Commands.deferredProxy(
                () -> subStateMachine.tryTargetState(subStateMachine, subIntake, subShooter, subTransfer,
                    subElevator))));

    readyToShoot.onTrue(
        Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble, constControllers.DRIVER_RUMBLE)).alongWith(
            Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, constControllers.OPERATOR_RUMBLE))))
        .onFalse(
            Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble, 0)).alongWith(
                Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0))));

    subDrivetrain.resetModulesToAbsolute();

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureTestBindings(conTestOperator);
  }

  private void configureDriverBindings(SN_XboxController controller) {
    controller.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    controller.btn_North.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[6].toPose2d())));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    // -- States --

    controller.btn_LeftTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer,
                subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    controller.btn_RightTrigger.whileTrue(
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.SHOOTING,
            subStateMachine, subDrivetrain, subElevator, subIntake, subTransfer, subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    controller.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SPEAKER, subStateMachine, subDrivetrain, subElevator,
                subIntake,
                subTransfer, subShooter)));

    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPIKE)))
        .onTrue(
            Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_SPIKE, subStateMachine, subDrivetrain,
                subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_A.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_AMP)))
        .onTrue(Commands
            .deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_AMP, subStateMachine, subDrivetrain,
                subElevator, subIntake, subTransfer, subShooter)));

    // "Unalive Shooter"

    controller.btn_B.onTrue(
        Commands.either(
            Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.STORE_FEEDER,
                subStateMachine, subDrivetrain, subElevator, subIntake, subTransfer, subShooter)),
            Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.NONE,
                subStateMachine, subDrivetrain, subElevator, subIntake, subTransfer, subShooter)),
            gamePieceTrigger)
            .alongWith(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_NONE))));

    controller.btn_West.whileTrue(Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.EJECTING, subStateMachine, subDrivetrain, subElevator, subIntake,
            subTransfer,
            subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subDrivetrain, subElevator, subIntake,
                subTransfer,
                subShooter)));

    controller.btn_LeftBumper.whileTrue(new ManualPivot(subShooter, controller.axis_RightY))
        .whileTrue(new ManualElevator(subElevator, controller.axis_LeftY));
  }

  private void configureTestBindings(SN_XboxController controller) {
    controller.btn_LeftTrigger.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.INTAKING)))
        .whileTrue(new Intaking(subStateMachine, subIntake, subShooter, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));

    controller.btn_RightTrigger.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.SHOOTING)))
        .whileTrue(new Shooting(subStateMachine, subElevator, subShooter, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));

    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.PREP_SHUFFLE)))
        .onTrue(new PrepTargetState(subStateMachine, subShooter, TargetState.PREP_SHUFFLE));

    controller.btn_A.onTrue(Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.AMP_POSITION)));
    controller.btn_Y.onTrue(Commands.runOnce(() -> subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT)));

    controller.btn_West.onTrue(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.EJECTING)))
        .whileTrue(new Ejecting(subStateMachine, subIntake, subTransfer))
        .onFalse(new NoneState(subStateMachine, subElevator, subIntake, subShooter, subTransfer));
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
    Command returnedCommand = new ParallelCommandGroup(
        // new
        // ZeroClimber(subClimber).withTimeout(constClimber.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroShooterPivot(subShooter).withTimeout(constShooter.ZEROING_TIMEOUT.in(Units.Seconds)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    returnedCommand.addRequirements(subStateMachine);
    return returnedCommand;
  };

}
