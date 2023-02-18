// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Balancer;
import frc.robot.commands.Conduire;
import frc.robot.commands.auto.TestAllerPoint;
import frc.robot.commands.auto.TrajetAuto;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CapteurCouleur;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {

  private final BasePilotable basePilotable = new BasePilotable();
  private final Limelight limelight = new Limelight();

  CommandXboxController pilote = new CommandXboxController(0);
  CapteurCouleur couleur = new CapteurCouleur();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    basePilotable.setDefaultCommand(new Conduire(pilote::getLeftY,pilote::getRightX, basePilotable));
  }

  
  private void configureBindings() {
    pilote.a().onTrue(new TestAllerPoint(basePilotable));
    pilote.x().whileTrue(new Balancer(basePilotable));
  }

  public Command getAutonomousCommand() {
   // return new RunCommand(() -> basePilotable.autoConduire(10, 10), basePilotable).until(basePilotable::isNotBalance).andThen(new BalancerPIDNormal(basePilotable));
 
  

    return new TrajetAuto(basePilotable);
  }
}