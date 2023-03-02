// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

public static final int kToleranceBalancer = 5;
public static final double kRamp = 0.2;


public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.75325); //détermier par sys id
    public static final double kSRamsete = 0.11258;
    public static final double kVRamsete = 2.2634;
    public static final double kARamsete = 0.33039;

    public static final double autoMaxVoltage = 10;

    public static final double kPRamsete = 2.7888;
    
    public static final SimpleMotorFeedforward driveTrainFeedFoward = new SimpleMotorFeedforward(Constants.kSRamsete, Constants.kVRamsete, Constants.kARamsete);

    public static final double maxVitesse = 3.0;
    public static final double maxAcceleration = 3.0;
}
