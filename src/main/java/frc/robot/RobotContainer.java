// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Conduire;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CapteurCouleur;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;



public class RobotContainer {

  private final BasePilotable basePilotable = new BasePilotable();
  //private final Limelight limelight = new Limelight();

  XboxController pilote = new XboxController(0);
  CapteurCouleur couleur = new CapteurCouleur();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    basePilotable.setDefaultCommand(new Conduire(pilote::getLeftY,pilote::getRightX, basePilotable));
  }

  
  private void configureBindings() {
    
  

  }

  
  public Command getAutonomousCommand() {
    return  new RunCommand(() -> basePilotable.autoConduire(0, 12), basePilotable);
  }
}
