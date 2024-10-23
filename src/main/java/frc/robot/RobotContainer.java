// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.frcteam3255.joystick.SN_XboxController;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constElevator;
import frc.robot.Constants.constField;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.AddVisionMeasurement;
import frc.robot.commands.Drive;
import frc.robot.commands.GamePieceRumble;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.Autos.Centerline;
import frc.robot.commands.Autos.PreloadOnly;
import frc.robot.commands.Autos.PreloadTaxi;
import frc.robot.commands.Autos.WingOnly;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.Zeroing.ZeroElevator;
import frc.robot.commands.Zeroing.ZeroShooterPivot;
import frc.robot.commands.States.IntakeSource;
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

  public final static StateMachine subStateMachine = new StateMachine();
  private final static Climber subClimber = new Climber();
  private final static Drivetrain subDrivetrain = new Drivetrain();
  private final static Elevator subElevator = new Elevator();
  private final static Intake subIntake = new Intake();
  private final static Shooter subShooter = new Shooter();
  private final static Transfer subTransfer = new Transfer();
  private final static Limelight subLimelight = new Limelight();

  private final Trigger gamePieceTrigger = new Trigger(() -> subTransfer.getGamePieceStored());

  private final BooleanSupplier readyToShoot = (() -> subDrivetrain.isDrivetrainFacingSpeaker()
      && subShooter.readyToShoot() && subStateMachine.isCurrentStateTargetState()
      && subTransfer.getGamePieceStored());

  private final IntakeSource comIntakeSource = new IntakeSource(subStateMachine, subShooter, subTransfer);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(
            new Drive(subDrivetrain, subStateMachine, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
                conDriver.btn_LeftBumper, conDriver.btn_RightBumper, conDriver.btn_RightTrigger,
                conDriver.btn_Y, conDriver.btn_B, conDriver.btn_A,
                new Trigger(() -> conDriver.btn_X.getAsBoolean() || conDriver.btn_LeftTrigger.getAsBoolean())));

    // - Manual Triggers -
    gamePieceTrigger
        .onTrue(Commands
            .deferredProxy(
                () -> subStateMachine.tryState(RobotState.STORE_FEEDER, subStateMachine, subClimber, subDrivetrain,
                    subElevator, subIntake, subTransfer, subShooter))
            .andThen(Commands.deferredProxy(
                () -> subStateMachine.tryTargetState(subStateMachine, subIntake, subShooter, subTransfer,
                    subElevator, subDrivetrain))))
        .onTrue(new GamePieceRumble(conDriver, conOperator).asProxy());

    new Trigger(readyToShoot).onTrue(
        Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble,
            constControllers.DRIVER_RUMBLE)).alongWith(
                Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble,
                    constControllers.OPERATOR_RUMBLE))))
        .onFalse(
            Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble,
                0)).alongWith(
                    Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0))));

    subDrivetrain.resetModulesToAbsolute();

    conDriver.setTriggerPressThreshold(0.2);

    NamedCommands.registerCommand("Intaking", Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber, subDrivetrain, subElevator,
            subIntake, subTransfer, subShooter))
        .until(gamePieceTrigger));

    SmartDashboard.putNumber("Preload Only Delay", 0);

    configureDriverBindings(conDriver);
    configureOperatorBindings(conOperator);
    configureTestBindings(conTestOperator);

    configureAutoSelector();
  }

  private void configureDriverBindings(SN_XboxController controller) {
    // Reset Pose
    controller.btn_North.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[6].toPose2d())));

    // Intake from source
    controller.btn_East.whileTrue(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.INTAKE_SOURCE,
        subStateMachine, subClimber, subDrivetrain, subElevator, subIntake, subTransfer, subShooter)))
        .onFalse(Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subClimber,
            subDrivetrain, subElevator, subIntake, subTransfer, subShooter))
            .unless(() -> comIntakeSource.getIntakeSourceGamePiece()));
  }

  private void configureOperatorBindings(SN_XboxController controller) {
    // -- States --
    // Intake
    controller.btn_LeftTrigger
        .whileTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.INTAKING, subStateMachine, subClimber, subDrivetrain, subElevator,
                subIntake,
                subTransfer,
                subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator,
                subIntake,
                subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    // Shoot
    controller.btn_RightTrigger.whileTrue(
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.SHOOTING,
            subStateMachine, subClimber, subDrivetrain, subElevator, subIntake, subTransfer, subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subClimber, subDrivetrain, subElevator,
                subIntake,
                subTransfer,
                subShooter))
            .unless(gamePieceTrigger));

    // Prep with vision
    controller.btn_RightBumper.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_VISION)))
        .onTrue(Commands
            .deferredProxy(
                () -> subStateMachine.tryState(RobotState.PREP_VISION, subStateMachine, subClimber, subDrivetrain,
                    subElevator, subIntake, subTransfer, subShooter)));

    // Ejecting
    controller.btn_LeftBumper.whileTrue(Commands.deferredProxy(
        () -> subStateMachine.tryState(RobotState.EJECTING, subStateMachine,
            subClimber, subDrivetrain, subElevator,
            subIntake,
            subTransfer,
            subShooter)))
        .onFalse(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.NONE, subStateMachine, subClimber,
                subDrivetrain, subElevator,
                subIntake,
                subTransfer,
                subShooter)));

    // Prep spike
    controller.btn_X.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPIKE)))
        .onTrue(
            Commands.deferredProxy(
                () -> subStateMachine.tryState(RobotState.PREP_SPIKE, subStateMachine, subClimber, subDrivetrain,
                    subElevator, subIntake, subTransfer, subShooter)));

    controller.btn_B.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_AMP)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_AMP, subStateMachine, subClimber, subDrivetrain, subElevator,
                subIntake, subTransfer, subShooter)));

    controller.btn_Y.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SUB_BACKWARDS)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SUB_BACKWARDS, subStateMachine, subClimber, subDrivetrain,
                subElevator,
                subIntake, subTransfer, subShooter)));

    // "Unalive Shooter"
    controller.btn_A.onTrue(
        Commands.deferredProxy(() -> subStateMachine.tryState(RobotState.PREP_NONE,
            subStateMachine, subClimber, subDrivetrain, subElevator, subIntake, subTransfer, subShooter))
            .alongWith(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_NONE))))
        .onFalse(Commands.runOnce(() -> subShooter.setShootingNeutralOutput()));

    // Prep subwoofer
    controller.btn_South.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SPEAKER)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SPEAKER, subStateMachine, subClimber, subDrivetrain,
                subElevator,
                subIntake, subTransfer, subShooter)));

    // Prep wing
    controller.btn_North.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_WING)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_WING, subStateMachine, subClimber, subDrivetrain,
                subElevator,
                subIntake, subTransfer, subShooter)));

    // Prep shuffle
    controller.btn_West.onTrue(Commands.runOnce(() -> subStateMachine.setTargetState(TargetState.PREP_SHUFFLE)))
        .onTrue(Commands.deferredProxy(
            () -> subStateMachine.tryState(RobotState.PREP_SHUFFLE, subStateMachine, subClimber, subDrivetrain,
                subElevator,
                subIntake, subTransfer, subShooter)));

    // Game Piece Override
    controller.btn_East.onTrue(Commands.runOnce(() -> subTransfer.setGamePieceCollected(true))
        .alongWith(Commands.runOnce(() -> subStateMachine.setRobotState(RobotState.STORE_FEEDER))));

    // Manual Shooter Pivot
    controller.btn_Back.whileTrue(new ManualPivot(subShooter, controller.axis_RightY));

    // Manual Elevator Pivot
    controller.btn_Start.whileTrue(new ManualElevator(subElevator, controller.axis_LeftY));

    // Zero Elevator and Climber
    controller.btn_LeftStick.onTrue(new ZeroElevator(subElevator));

    // Zero Shooter
    controller.btn_RightStick.onTrue(new ZeroShooterPivot(subShooter));
  }

  private void configureTestBindings(SN_XboxController controller) {

    controller.btn_LeftBumper.onTrue(Commands.runOnce(SignalLogger::start));
    controller.btn_RightBumper.onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */
    controller.btn_Y.whileTrue(subShooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller.btn_A.whileTrue(subShooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller.btn_B.whileTrue(subShooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller.btn_X.whileTrue(subShooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void configureAutoSelector() {
    DoubleSupplier preloadDelay = () -> SmartDashboard.getNumber("Preload Only Auto", 0);

    // -- Preload Sub --
    autoChooser.addOption("Preload Only Amp-Side", new PreloadOnly(subStateMachine, subClimber, subDrivetrain,
        subElevator, subIntake, subShooter, subTransfer, 0, preloadDelay));
    autoChooser.setDefaultOption("Preload Only Center",
        new PreloadOnly(subStateMachine, subClimber, subDrivetrain, subElevator,
            subIntake, subShooter, subTransfer,
            1, preloadDelay));
    autoChooser.addOption("Preload Only Source-Side", new PreloadOnly(subStateMachine, subClimber, subDrivetrain,
        subElevator, subIntake, subShooter, subTransfer, 2, preloadDelay));

    autoChooser.addOption("Preload Taxi",
        new PreloadTaxi(subStateMachine, subClimber, subDrivetrain, subElevator,
            subIntake, subShooter, subTransfer));
    autoChooser.addOption("Wing Only Down", new WingOnly(subStateMachine,
        subClimber, subDrivetrain, subElevator,
        subIntake, subTransfer, subShooter, readyToShoot, true));
    autoChooser.addOption("Wing Only Up", new WingOnly(subStateMachine,
        subClimber, subDrivetrain, subElevator,
        subIntake, subTransfer, subShooter, readyToShoot, false));

    autoChooser.addOption("Centerline :3", new Centerline(subStateMachine,
        subClimber, subDrivetrain, subElevator,
        subIntake, subTransfer, subShooter, readyToShoot, false));

    SmartDashboard.putData(autoChooser);
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
        new ZeroElevator(subElevator).withTimeout(constElevator.ZEROING_TIMEOUT.in(Units.Seconds)),
        new ZeroShooterPivot(subShooter).withTimeout(constShooter.ZEROING_TIMEOUT.in(Units.Seconds)))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    returnedCommand.addRequirements(subStateMachine);
    return returnedCommand;
  }

  public static Command AddVisionMeasurement() {
    return new AddVisionMeasurement(subDrivetrain, subLimelight)
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
  }

}
