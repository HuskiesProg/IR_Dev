// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable;

public class TestAllerPoint extends SequentialCommandGroup {
  
  public TestAllerPoint(BasePilotable basePilotable) {
    PathPlannerTrajectory trajet = basePilotable.creerTrajectoire(1, 0, 0);
    addCommands(
    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(false)),
   
    basePilotable.ramsetePathPlanner(trajet),

    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(true)));
  }
}
