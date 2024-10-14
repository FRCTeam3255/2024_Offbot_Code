// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GamePieceRumble extends SequentialCommandGroup {
  double startTime = 0;

  public GamePieceRumble(SN_XboxController conDriver, SN_XboxController conOperator) {

    addCommands(
        Commands.runOnce(() -> startTime = Timer.getFPGATimestamp()),

        Commands.runOnce(
            () -> conDriver.setRumble(RumbleType.kBothRumble, 1))
            .alongWith(
                Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 1))),

        Commands.waitSeconds(0.1),

        Commands.runOnce(
            () -> conDriver.setRumble(RumbleType.kBothRumble, 0.5))
            .alongWith(
                Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0.5))),

        Commands.waitSeconds(0.1),

        Commands.runOnce(
            () -> conDriver.setRumble(RumbleType.kBothRumble, 1))
            .alongWith(
                Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 1))),

        Commands.waitSeconds(0.1),

        Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble, 0)),
        Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0)));
  }
}
