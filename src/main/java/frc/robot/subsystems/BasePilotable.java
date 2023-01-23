// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;


import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

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

  

  /** Creates a new BasePilotable. */
  public BasePilotable() {
   

    resetEncodeur();
    resetGyro();
    conversionEncodeur=1; // À VALIDER Math.PI*0.1865/(256*3*2.5); //roue de 18.46 cm déterminé manuellement, ratio 2.5:1 shaft-roue 3:1 encodeur-shaft encodeur 256 clic encodeur 
    
    setRamp(0);
    encodeurg.setDistancePerPulse(conversionEncodeur);
    encodeurd.setDistancePerPulse(conversionEncodeur);
    setNeutralMode(IdleMode.kCoast);
    //odometrie= new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    neog.setInverted(true);
    neod.setInverted(false);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   
  }


////////Méthode pour conduire
  public void conduire(double vx, double vz) {
    drive.arcadeDrive(-vx, -0.7*vz);// on doit invert les deux axes a cause de 2023 donc ajouter un - a vz
  
  }
  
  public void autoConduire(double leftVolts, double rightVolts) {
    neog.setVoltage(leftVolts);
    neod.setVoltage(rightVolts);
    drive.feed();
  }

  /////Méthodes pour configurer les moteurs

  public void setNeutralMode(IdleMode mode){
    neog1.setIdleMode(mode);
    neog2.setIdleMode(mode);
    neod1.setIdleMode(mode);
    neod2.setIdleMode(mode);
  }
  
  public void setRamp(double ramp) {
    neog1.setOpenLoopRampRate(ramp);
    neog2.setOpenLoopRampRate(ramp);
    neod1.setOpenLoopRampRate(ramp);
    neod2.setOpenLoopRampRate(ramp);
  }
  
  

  ////////Méthode encodeur

  public double getPositionD() {
    return encodeurd.getDistance();
  }

  public double getPositionG() {
    return encodeurg.getDistance();
  }

  public double getVitesseD() {
    return encodeurd.getRate();
  }
  
  public double getVitesseG() {
    return encodeurg.getRate();
  }
  public double getVitesse() {
    return (getVitesseD() + getVitesseG()) / 2;
  }

  public void resetEncodeur() {
    encodeurd.reset();
    encodeurg.reset();
    neog1.getEncoder().setPosition(0);
  }


  /////Méthode GYRO Pigeon
  public double getYaw() {//en z
    return gyro.getYaw();
    
  }
  public double getRoll() {///à changer pour pitch
    return gyro.getRoll();


  }

  public void resetGyro() {
    gyro.setYaw(0);
  }



}



