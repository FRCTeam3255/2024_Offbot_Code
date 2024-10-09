// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_XboxController;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GamePieceRumble extends SequentialCommandGroup {
  double startTime = 0;
  Measure<Time> duration = Units.Seconds.of(1);

  public GamePieceRumble(SN_XboxController conDriver, SN_XboxController conOperator) {

    addCommands(
        Commands.runOnce(() -> startTime = Timer.getFPGATimestamp()),
        Commands.run(
            () -> conDriver.setRumble(RumbleType.kBothRumble, Math.cosh((Timer.getFPGATimestamp() - startTime) * 2)))
            .alongWith(
                Commands.run(() -> conOperator.setRumble(RumbleType.kBothRumble,
                    Math.cosh((Timer.getFPGATimestamp() - startTime) * 2)))),
        Commands.waitSeconds(duration.in(Units.Seconds)),

        Commands.runOnce(() -> conDriver.setRumble(RumbleType.kBothRumble, 0)),
        Commands.runOnce(() -> conOperator.setRumble(RumbleType.kBothRumble, 0)));
  }
}
