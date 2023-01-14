// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasePilotable extends SubsystemBase {

  private CANSparkMax neog1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax neog2 = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax neod1 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax neod2 = new CANSparkMax(33, MotorType.kBrushless);

  private MotorControllerGroup neog = new MotorControllerGroup(neog1, neog2);
  private MotorControllerGroup neod = new MotorControllerGroup(neod1, neod2);

  private DifferentialDrive drive = new DifferentialDrive(neog, neod);

  private Encoder encodeurg = new Encoder(0, 1, false);
  private Encoder encodeurd = new Encoder(2, 3, true);
  private double conversionEncodeur;

  private PigeonIMU gyro = new PigeonIMU(3);

  private DifferentialDriveOdometry odometrie;

  
  // Capteur de couleur
  ColorSensorV3 capteurCouleur = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kPurpleTarget = new Color(0.189, 0.439, 0.372); // mauve = 310 720 610 1640

  /** Creates a new BasePilotable. */
  public BasePilotable() {

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Capteur de couleur valeur
    SmartDashboard.putNumber("rouge", capteurCouleur.getRed());
    SmartDashboard.putNumber("bleu", capteurCouleur.getBlue());
    SmartDashboard.putNumber("vert", capteurCouleur.getGreen());
    SmartDashboard.putNumber("Proximité", capteurCouleur.getProximity());
    SmartDashboard.putNumber("Ir", capteurCouleur.getIR());

    if (capteurCouleur.getProximity() > 100) {

    }

    // Color detected Yellow or Purple
    String colorString;
    Color detectedColor = capteurCouleur.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kPurpleTarget) {
      colorString = "Papaul";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putString("Detected Color", colorString);

  }
}
