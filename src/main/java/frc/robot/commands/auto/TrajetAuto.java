// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable; 

public class TrajetAuto extends SequentialCommandGroup {
  
  public TrajetAuto(BasePilotable basePilotable) {
    Trajectory trajet = basePilotable.creerTrajectoire("trajet_en_s");
    
    
    addCommands(
    new InstantCommand(() -> basePilotable.resetOdometry(trajet.getInitialPose())),
    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(false)),
    basePilotable.ramseteSimple(trajet),

    new InstantCommand(() -> basePilotable.setBrakeEtRampTeleop(true)));
  }
}
