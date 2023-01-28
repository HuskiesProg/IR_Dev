// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Balancer extends ProfiledPIDCommand {
  /** Creates a new Balancer. */
  public Balancer(BasePilotable basePilotable) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            -1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(40, 20)),
        // This should return the measurement
        basePilotable::getRoll,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> basePilotable.autoConduire(output, output), 
          // Use the output (and setpoint, if desired) here
          basePilotable);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().setTolerance(Constants.kToleranceBalancer);
    basePilotable.setBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
