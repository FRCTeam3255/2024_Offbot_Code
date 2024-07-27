// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constControllers;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final SN_XboxController conDriver = new SN_XboxController(mapControllers.DRIVER_USB);
  private final SN_XboxController conOperator = new SN_XboxController(mapControllers.OPERATOR_USB);

  private final Drivetrain subDrivetrain = new Drivetrain();

  public RobotContainer() {
    conDriver.setLeftDeadband(constControllers.DRIVER_LEFT_STICK_DEADBAND);

    subDrivetrain
        .setDefaultCommand(new Drive(subDrivetrain, conDriver.axis_LeftY, conDriver.axis_LeftX, conDriver.axis_RightX,
            conDriver.btn_LeftBumper,
            conDriver.btn_Y, conDriver.btn_B, conDriver.btn_A, conDriver.btn_X,
            conDriver.btn_LeftTrigger, conDriver.btn_RightTrigger));

    configureBindings();

    subDrivetrain.resetModulesToAbsolute();
  }

  private void configureBindings() {
    conDriver.btn_B.onTrue(Commands.runOnce(() -> subDrivetrain.resetModulesToAbsolute()));

    // TODO: This is a workaround that will be implemented properly in SuperCORE
    conDriver.btn_Back.onTrue(
        Commands.runOnce(() -> subDrivetrain.resetDriving(() -> constField.isRedAlliance()))
            .andThen(Commands
                .runOnce(() -> subDrivetrain.resetPoseToPose(constField.getFieldPositions().get()[6].toPose2d()))));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
