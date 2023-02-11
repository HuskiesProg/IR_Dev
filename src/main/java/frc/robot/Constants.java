// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

public static final int kToleranceBalancer = 5;
public static final double rampTeleop = 0.2;

public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.87002); //détermier par sys id
    public static final double kSRamsete = 0.065517;
    public static final double kVRamsete = 2.8641;
    public static final double kARamsete = 4.4822;

    public static final double autoMaxVoltage = 10;

    public static final double kPRamsete = 0.067791;
    
    public static final SimpleMotorFeedforward driveTrainFeedFoward = new SimpleMotorFeedforward(Constants.kSRamsete, Constants.kVRamsete, Constants.kARamsete);

    public static final double kRamp = 0.2;
    
    public static final double maxVitesse = 1.5;
    public static final double maxAcceleration = 0.75;
}
