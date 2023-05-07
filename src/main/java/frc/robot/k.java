// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Lib.SwerveData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
  public static class OI {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;
  }
  public static class SWERVE{
    
    public static final double wheelDiameter_M = 0.1039;  // Measured in CAD
    public static final double wheelCircumference_M = Math.PI * wheelDiameter_M;  // PI * Diameter in M

    public static final double falconMaxRPS = 106.33;  // 6380/60

    public static final double steerGearRatio = 15.42857;
    public static final double driveGearRatio = 7.84615;

    public static final double driveMotor_MPR = wheelCircumference_M / driveGearRatio;
    public static final double driveMax_MPS = (falconMaxRPS/driveGearRatio) * wheelCircumference_M; // 4.38 MPS or 14.337 ft/sec

    public static final double robotSwerveCircumference_M = 1.712;
    public static final double robotMaxRotationSpeed_RadPS = 2*Math.PI * (driveMax_MPS/robotSwerveCircumference_M);



    public static SwerveData FL = new SwerveData(
        "FL", 0, InvertedValue.Clockwise_Positive, 0, InvertedValue.Clockwise_Positive, 0, SensorDirectionValue.Clockwise_Positive, 0);
    public static SwerveData FR = new SwerveData(
        "FR", 0, InvertedValue.Clockwise_Positive, 0, InvertedValue.Clockwise_Positive, 0, SensorDirectionValue.Clockwise_Positive, 0);
        public static SwerveData B = new SwerveData(
            "B", 0, InvertedValue.Clockwise_Positive, 0, InvertedValue.Clockwise_Positive, 0, SensorDirectionValue.Clockwise_Positive, 0);
  }
  public static class DRIVETRAIN{
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.135, 0.235 ),
      new Translation2d(0.135, -0.235),
      new Translation2d(-0.272, 0.0)
    );
    public static int gyroCANId;

  }
  public static class ROBOT {
    public static double MaxBatteryVoltage = 12.0;
  }
}
