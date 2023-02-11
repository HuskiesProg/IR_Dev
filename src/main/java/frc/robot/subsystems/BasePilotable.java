// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BasePilotable extends SubsystemBase {
  //Moteurs
  private CANSparkMax neog1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax neog2 = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax neod1 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax neod2 = new CANSparkMax(33, MotorType.kBrushless);
  
  private MotorControllerGroup neog = new MotorControllerGroup(neog1, neog2);
  private MotorControllerGroup neod = new MotorControllerGroup(neod1, neod2);

  private DifferentialDrive drive = new DifferentialDrive(neog, neod);

  //Encodeurs
  private Encoder encodeurG = new Encoder(0, 1, false);
  private Encoder encodeurD = new Encoder(2, 3, true);
  private double conversionEncodeur;

  //Gyro
  private PigeonIMU gyro = new PigeonIMU(3);
  private double rollOffset = 0;
  

  //Odometry
  private DifferentialDriveOdometry odometry;

  //PID Balancer
  private PIDController pidBalancer = new PIDController(-0.5, 0, 0);

  public BasePilotable() {
    //Reset initiaux
    resetEncodeur();
    resetGyro();

    //Configuration des encodeurs externes
    conversionEncodeur = Math.PI*0.1865/(256*3*2.5); //roue de 18.46 cm déterminé manuellement, ratio 2.5:1 shaft-roue 3:1 encodeur-shaft encodeur 256 clic encodeur circonférence de la roue 58cm aproximatif
    encodeurG.setDistancePerPulse(conversionEncodeur);
    encodeurD.setDistancePerPulse(conversionEncodeur);
    neog.setInverted(true);
    neod.setInverted(false);
    
    //Ramp et Brake
    setRamp(0);
    setBrake(false);

    //Odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD());

    pidBalancer.setSetpoint(0);
    
    pidBalancer.setTolerance(2);
  }
  
  @Override
  public void periodic() {

    odometry.update(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD());

    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("yaw", getYaw());
    SmartDashboard.putNumber("roll", getRoll());
    SmartDashboard.putNumber("Tolerance", pidBalancer.getPositionTolerance());
    SmartDashboard.putNumber("P", pidBalancer.getP());
    SmartDashboard.putNumber("Position Error", pidBalancer.getPositionError());

    SmartDashboard.putNumber("Position gauche", getPositionG());
    SmartDashboard.putNumber("Position droite", getPositionD());
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

  public void setBrake(Boolean brake){
    if (brake) {
    neog1.setIdleMode(IdleMode.kBrake);
    neog2.setIdleMode(IdleMode.kBrake);
    neod1.setIdleMode(IdleMode.kBrake);
    neod2.setIdleMode(IdleMode.kBrake);
    }

    else {
      neog1.setIdleMode(IdleMode.kCoast);
      neog2.setIdleMode(IdleMode.kCoast);
      neod1.setIdleMode(IdleMode.kCoast);
      neod2.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public void setRamp(double ramp) {
    neog1.setOpenLoopRampRate(ramp);
    neog2.setOpenLoopRampRate(ramp);
    neod1.setOpenLoopRampRate(ramp);
    neod2.setOpenLoopRampRate(ramp);
  }
  
  

  ////////Méthode encodeur

  public double getPositionD() {
    return encodeurD.getDistance();
  }

  public double getPositionG() {
    return encodeurG.getDistance();
  }

  public double getVitesseD() {
    return encodeurD.getRate();
  }
  
  public double getVitesseG() {
    return encodeurG.getRate();
  }
  public double getVitesse() {
    return (getVitesseD() + getVitesseG()) / 2;
  }

  public void resetEncodeur() {
    encodeurD.reset();
    encodeurG.reset();
    neog1.getEncoder().setPosition(0);
  }


  /////Méthode GYRO Pigeon
public double getAngle() {
  return gyro.getYaw();
}

public void resetGyro() {
  gyro.setYaw(0);
  rollOffset = gyro.getRoll();
  
}


  public double getYaw() {//en z
    return gyro.getYaw();
    
  }
  
  public double getRoll() {///à changer pour pitch dans charged up selon le sens du gyro
    return -(gyro.getRoll() - rollOffset);
  }


  public boolean isNotBalance(){
    return Math.abs(getRoll()) >= Constants.kToleranceBalancer;
  }
//////////////////////////Odométrie
  public double[] getOdometry(){
    double[] position = new double[3];
    double x = getPose().getTranslation().getX();
    double y = getPose().getTranslation().getY();
    double theta = getPose().getRotation().getDegrees();
    position[0] = x;
    position[1] = y;
    position[2] = theta;
    return position;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry() {
    resetEncodeur();
    resetGyro();
    odometry.resetPosition(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD(), getPose());
  }

  public double voltagePIDBalancer() {
    return pidBalancer.calculate(getRoll(), 0);
  }

  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getVitesseG(), getVitesseD());
  }

public Trajectory creerTrajectoire(String trajet)
{

  String trajetJSON = "output/"+trajet+".wpilib.json";
  try
  {
    var path = Filesystem.getDeployDirectory().toPath().resolve(trajetJSON);
    return TrajectoryUtil.fromPathweaverJson(path);
  }
  catch (IOException e)
  {
    DriverStation.reportError( "Unable to open trajectory :" + trajetJSON, e.getStackTrace() );
    return null;
  }

}


  public boolean isBalancer() {
    return pidBalancer.atSetpoint();
  }

  public Command ramseteSimple(Trajectory trajectoire)
  {
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectoire,
      this::getPose,
      new RamseteController(2, 0.7),
      new SimpleMotorFeedforward(Constants.kSRamsete, Constants.kVRamsete, Constants.kARamsete),
      Constants.kinematics,
      this::getWheelSpeeds,
      new PIDController(Constants.kPRamsete, 0, 0),
      new PIDController(Constants.kPRamsete, 0, 0),
      this::autoConduire,
      this);
      return ramseteCommand.andThen(()->autoConduire(0, 0));
  }
 }



