// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WingOnly extends SequentialCommandGroup {
  Drivetrain subDrivetrain;
  boolean goesDown;

  /** Creates a new WingDown. */
  public WingOnly(Drivetrain subDrivetrain, boolean goesDown) {
    this.subDrivetrain = subDrivetrain;
    this.goesDown = goesDown;

    addCommands(
        // just for testing the whole path
        new PathPlannerAuto("PsW1sW2sW3s")

    // Intake

    // Drive to first note
    // new PathPlannerAuto(determinePathName() + ".1"),

    // Transfer + Shoot

    // Intake

    // Drive to second note
    // new PathPlannerAuto(determinePathName() + ".2"),

    // Transfer + Shoot

    // Intake

    // Drive to third note
    // new PathPlannerAuto(determinePathName() + ".3")

    // Transfer + Shoot
    );
  }

  public String determinePathName() {
    return (goesDown) ? "PsW1sW2sW3s" : "PsW3sW2sW1s";
  }

}
