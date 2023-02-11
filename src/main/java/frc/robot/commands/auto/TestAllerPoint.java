// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAllerPoint extends SequentialCommandGroup {
  /** Creates a new TestAllerPoint. */
  public TestAllerPoint(BasePilotable basePilotable) {
    Trajectory trajet = basePilotable.creerTrajectoire(1, 0, 0);
    addCommands(
    new InstantCommand(() -> basePilotable.setRamp(0)),
    new InstantCommand(() -> basePilotable.setBrake(true)),
    basePilotable.ramseteSimple(trajet),

    new InstantCommand(() -> basePilotable.setBrake(false)),
    new InstantCommand(() -> basePilotable.setRamp(Constants.kRamp)));
  }
}
