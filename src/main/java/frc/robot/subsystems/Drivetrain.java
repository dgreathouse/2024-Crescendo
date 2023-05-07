// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.k;
import frc.robot.Lib.SwerveModule;

public class Drivetrain extends SubsystemBase {
    SwerveModule m_fl = new SwerveModule(k.SWERVE.FL);
    SwerveModule m_fr = new SwerveModule(k.SWERVE.FR);
    SwerveModule m_b = new SwerveModule(k.SWERVE.B);

    PIDController m_xPID = new PIDController(0, 0, 0);
    PIDController m_yPID = new PIDController(0, 0, 0);
    ProfiledPIDController m_thetaPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    HolonomicDriveController m_AutoDriveController = new HolonomicDriveController(m_xPID, m_yPID, m_thetaPID);

    Pigeon2 m_gyro = new Pigeon2(k.DRIVETRAIN.gyroCANId);
    

    public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        k.DRIVETRAIN.kinematics, 
        m_gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            m_fl.getPosition(),
            m_fr.getPosition(),
            m_b.getPosition()
    });
    /** Creates a new Drivetrain. */
    public Drivetrain() {
        
    }
    /** Drive the serve modules with thumbstick inputs. This is always Optimised and FieldRelative
     * 
     * @param _xSpeed Forward/Reverse speed +/- 1.0 
     * @param _ySpeed Left/Right speed +/- 1.0
     * @param _rot CCW/CW speed +/- 1.0
     */
    public void drive(double _xSpeed, double _ySpeed, double _rot){

        // Calculate the Chassis Speeds from the thumbstick inputs
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(_xSpeed * k.SWERVE.driveMax_MPS , _ySpeed * k.SWERVE.driveMax_MPS, _rot * k.SWERVE.robotMaxRotationSpeed_RadPS, m_gyro.getRotation2d());
        // Calculate the individual wheel speeds and angles
        var swerveModuleStates = k.DRIVETRAIN.kinematics.toSwerveModuleStates(speeds);  
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speeds, k.SWERVE.driveMax_MPS, k.SWERVE.driveMax_MPS, k.SWERVE.robotMaxRotationSpeed_RadPS);
        
    }
    /**
     * Uses the HolonomicDriveController to follow tragectories
     */
    public void driveAuto(){

    }

    @Override
    public void periodic() {
        m_odometry.update(
            m_gyro.getRotation2d(),  
            new SwerveModulePosition[] {
                m_fl.getPosition(),
                m_fr.getPosition(),
                m_b.getPosition()
    });
        // This method will be called once per scheduler run
    }
}
