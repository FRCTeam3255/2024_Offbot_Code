// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.commands.Autos.WingOnly;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new Drive(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
            conDriver.btn_A));

    configureBindings();
    configureAutoSelector();

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));
    conDriver.btn_Start.onTrue(Commands.runOnce(() -> subDrivetrain.resetYaw(180)));
    conDriver.btn_Back.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(180)))));
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("Wing Only Down", new WingOnly(subDrivetrain, true));
    autoChooser.addOption("Wing Only Up", new WingOnly(subDrivetrain, false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
