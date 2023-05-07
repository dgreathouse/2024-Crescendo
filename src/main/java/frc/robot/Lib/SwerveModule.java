// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;


import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.MagnetSensorConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.k;


public class SwerveModule {
    private TalonFX m_steer;
    private TalonFX m_drive;
    private CANcoder m_encoder;
    private SwerveData m_data;

    VoltageOut v = new VoltageOut(0);

    public SwerveModule(SwerveData _data){
        m_data = _data;
        m_steer = new TalonFX(_data.steerCANId, "CANivore");
        m_drive = new TalonFX(_data.driveCANId, "CANivore");
        m_encoder = new CANcoder(m_data.encoderCANId, "CANivore");
        
        // Ensure the configurations are default at start up
        m_steer.getConfigurator().apply(new TalonFXConfiguration());
        m_drive.getConfigurator().apply(new TalonFXConfiguration());
        m_encoder.getConfigurator().apply(new CANcoderConfiguration());

        var driveMotorConfig = new MotorOutputConfigs();
        driveMotorConfig.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Inverted = m_data.driveInvertValue;
        m_drive.getConfigurator().apply(driveMotorConfig);

        var steerMotorConfig = new MotorOutputConfigs();
        steerMotorConfig.NeutralMode = NeutralModeValue.Brake;
        steerMotorConfig.Inverted = m_data.steerInvertValue;
        m_steer.getConfigurator().apply(steerMotorConfig);

        m_encoder.getAbsolutePosition().setUpdateFrequency(100);
        var encoderConfig = new MagnetSensorConfigs();
        encoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderConfig.SensorDirection = m_data.encoderDirection;
        m_encoder.getConfigurator().apply(encoderConfig);
   
        
    }
    /******************** Steer Motor information ************************/
    /**
     * 
     * @return The value of the absolute encoder in degrees
     */
    public double getEncoderAbsolutePosition(){
        return m_encoder.getAbsolutePosition().getValue().doubleValue() * 360.0;
    }
    /**
     * 
     * @return The Steer motor position of the wheel in degrees
     */
    public double getSteerPosition(){
        return m_steer.getPosition().getValue().doubleValue() / k.SWERVE.steerGearRatio;
    }
        /**
     * 
     * @return the steer motor velocity in degrees/sec
     */
    public double getSteerVelocity(){
        return m_steer.getVelocity().getValue() * 360.0;
    }
    /**
        Reset the steer position to the offset from the absolute encoder.
        This is to be called during initialization when the motor is inactive.
     */
    public void resetSteerPosition(){
        // Motor is in rotations and gear ratio
        m_steer.setRotorPosition((((getEncoderAbsolutePosition()) - m_data.steerAngleOffset) / 360)  * k.SWERVE.steerGearRatio);
    }



    /******************** Drive Motor information ************************/
    /**
     * 
     * @return The drive position in meters
     */
    public double getDrivePosition(){
        return m_drive.getPosition().getValue() * k.SWERVE.driveMotor_MPR;
    }
    /**
     * 
     * @return The drive velocity in degrees/sec
     */
    public double getDriveVelocity(){
        return m_drive.getVelocity().getValue().doubleValue() * 360;
    }
    /**
     * 
     * @param _val The position of the motor to set in rotations.
     * 
     * This is generally used to set a value of zero and reset the motor position.
     */
    public void setDrivePosition(double _val){
        m_drive.setRotorPosition(_val);
    }

    /******************** Swerve Moodule information ************************/
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(),new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /********************* Drive the modules ********************************/
    public void setDesiredState(SwerveModuleState _desiredState){
        SwerveModuleState state = SwerveModuleState.optimize(_desiredState, new Rotation2d(Math.toRadians(getSteerPosition())));
        
       // m_drive.setVoltage((state.speedMetersPerSecond/k.SWERVE.driveMax_MPS) * k.ROBOT.MaxBatteryVoltage);
        m_drive.setControl(v.withEnableFOC(true).withOutput((state.speedMetersPerSecond/k.SWERVE.driveMax_MPS) * k.ROBOT.MaxBatteryVoltage));
        
       
    }

}
