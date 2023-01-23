// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CapteurCouleur extends SubsystemBase {

  // Capteur de couleur
  ColorSensorV3 capteurCouleur = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();


  public CapteurCouleur() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Capteur de couleur valeur
     SmartDashboard.putNumber("rouge", capteurCouleur.getRed());
     SmartDashboard.putNumber("bleu", capteurCouleur.getBlue());
     SmartDashboard.putNumber("vert", capteurCouleur.getGreen());
     SmartDashboard.putNumber("Proximité", capteurCouleur.getProximity());

     if (isInRange(capteurCouleur.getRed(), 9, 6) &&
      isInRange(capteurCouleur.getGreen(), 9, 6) &&
      isInRange(capteurCouleur.getBlue(), 7, 4))
    {
      SmartDashboard.putString("detection", "cube");
    }

    else if (isInRange(capteurCouleur.getRed(), 17, 5) &&
    isInRange(capteurCouleur.getGreen(), 20, 5) &&
    isInRange(capteurCouleur.getBlue(), 9, 4))
    
    {
      SmartDashboard.putString("detection", "cube");
    }

    else if (isInRange(capteurCouleur.getRed(), 55, 20) &&
    isInRange(capteurCouleur.getGreen(), 55, 20) &&
    isInRange(capteurCouleur.getBlue(),10, 6))

    {
      SmartDashboard.putString("detection", "cone");
    }
 
    else 
    {
      SmartDashboard.putString("detection", "rien");
    }

     // Color detected Yellow or Purple
     String colorString;
     Color detectedColor = capteurCouleur.getColor();
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  }

  private boolean isInRange(int valeurTest, int cible, int range) {
    return (valeurTest > (cible - range)) && (valeurTest < (cible + range));
  }
}
