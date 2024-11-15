// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.StateMachine.RobotStateInterface;
import java.util.HashMap;
import java.util.List;

public abstract class StatefulCommand extends Command {

  /**
   * @return The state that the robot is in when this command is running
   */
  protected abstract RobotStateInterface getState();

  protected abstract HashMap<RobotStateInterface, List<RobotStateInterface>> getTransitions();

  @Override
  public void schedule() {
    if (getTransitions().get(getState()).contains(RobotContainer.subStateMachine.getRobotState())) {
      super.schedule();
    } else {
      RobotContainer.subStateMachine.setQueueState(getState());
      Commands.print("ITS SO OVER D: Invalid State Transition :3").schedule();
    }
  }

}