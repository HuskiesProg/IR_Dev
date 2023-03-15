// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable; 

public class TrajetAutoPathPlanner extends SequentialCommandGroup {
  
  public TrajetAutoPathPlanner(BasePilotable basePilotable) {
    PathPlannerTrajectory trajet1 = basePilotable.creerTrajectoirePathPlanner("CoopChercherCone", true);
    PathPlannerTrajectory trajet2 = basePilotable.creerTrajectoirePathPlanner("PlacerConeBande", true);
    
    
    addCommands(
    new InstantCommand(() -> basePilotable.placerRobotPositionInitial(trajet1);),
    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(false)),
    basePilotable.ramsetePathPlanner(trajet1),
    basePilotable.ramsetePathPlanner(trajet2),
  
    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(true)));
  }
}
