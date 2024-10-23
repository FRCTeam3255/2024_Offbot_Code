// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constField;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestL extends SequentialCommandGroup {
  Drivetrain subDrivetrain;

  String pathName = "testSpin";

  /** Creates a new PreloadTaxi. */
  public TestL(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    addCommands(
        Commands.runOnce(() -> subDrivetrain.resetPoseToPose(
            getInitialPose().get())),

        new PathPlannerAuto(pathName));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!constField.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(pathName)
        : PathPlannerPath.fromPathFile(pathName).flipPath().getPreviewStartingHolonomicPose();
  }

}
