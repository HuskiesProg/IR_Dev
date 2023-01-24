// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;
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
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kPurpleTarget = new Color(0.189, 0.439, 0.372);


  public CapteurCouleur() {
    colorMatcher.addColorMatch(kPurpleTarget);
    colorMatcher.addColorMatch(kYellowTarget);

  }

  @Override
  public void periodic() {

    String colorString;
    Color detection = capteurCouleur.getColor();
    ColorMatchResult matcher = colorMatcher.matchClosestColor(detection);
    
     if (matcher.color == kPurpleTarget) {
      colorString = "Purple";
    } else if (matcher.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("rouge", capteurCouleur.getRed());
    SmartDashboard.putNumber("bleu", capteurCouleur.getBlue());
    SmartDashboard.putNumber("vert", capteurCouleur.getGreen());
    SmartDashboard.putNumber("Proximité", capteurCouleur.getProximity());

    // This method will be called once per scheduler run
     // Capteur de couleur valeur
     

/* 
     SmartDashboard.putNumber("rouge", capteurCouleur.getRed());
     SmartDashboard.putNumber("bleu", capteurCouleur.getBlue());
     SmartDashboard.putNumber("vert", capteurCouleur.getGreen());
     SmartDashboard.putNumber("Proximité", capteurCouleur.getProximity());

if (isInRange(capteurCouleur.getRed(), 15, 6) &&
      isInRange(capteurCouleur.getGreen(), 17, 8) &&
      isInRange(capteurCouleur.getBlue(), 15, 6))
    {
      SmartDashboard.putString("detection", "cube");
    }

    else if (isInRange(capteurCouleur.getRed(), 31, 13) &&
    isInRange(capteurCouleur.getGreen(), 40, 13) &&
    isInRange(capteurCouleur.getBlue(), 20, 9))
    
    {
      SmartDashboard.putString("detection", "cube");
    }

    else if (isInRange(capteurCouleur.getRed(), 150, 100) &&
    isInRange(capteurCouleur.getGreen(), 170, 100) &&
    isInRange(capteurCouleur.getBlue(),30, 20))

    {
      SmartDashboard.putString("detection", "cone");
    }
 
    else 
    {
      SmartDashboard.putString("detection", "rien");
    } 
    
     String colorString;
     Color detectedColor = capteurCouleur.getColor();
     ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    */
    

     // Color detected Yellow or Purple

    /*SmartDashboard.putNumber("rouge", capteurCouleur.getRed());
    SmartDashboard.putNumber("bleu", capteurCouleur.getBlue());
    SmartDashboard.putNumber("vert", capteurCouleur.getGreen());
    SmartDashboard.putNumber("Proximité", capteurCouleur.getProximity());
    SmartDashboard.putNumber("Ir", capteurCouleur.getIR());

    if (capteurCouleur.getProximity() > 100) {

    }

    // Color detected Yellow or Purple
    String colorString;
    Color detectedColor = capteurCouleur.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kPurpleTarget) {
      colorString = "Papaul";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putString("Detected Color", colorString);
  */}

  /*private boolean isInRange(int valeurTest, int cible, int range) {
    return (valeurTest > (cible - range)) && (valeurTest < (cible + range));
  }*/
}
