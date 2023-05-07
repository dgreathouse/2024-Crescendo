// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

/** Add your docs here. */
public class SwerveData {

    public String name = "";
    public int steerCANId = 0;
    public int driveCANId = 0;
    public int encoderCANId = 0;
    public InvertedValue driveInvertValue = InvertedValue.Clockwise_Positive;
    public InvertedValue steerInvertValue = InvertedValue.Clockwise_Positive;
    public SensorDirectionValue encoderDirection = SensorDirectionValue.Clockwise_Positive;
    public double steerAngleOffset = 0;
    /***
     * 
     * @param _name                 String name of this device
     * @param _driveCANId           Drive CAN Id
     * @param _driveInvertValue     Drive Invert of Motor
     * @param _steerCANId           Steer CAN Id
     * @param _steerInvertValue     Steer Invert of Motor
     * @param _encoderCANId         Encoder CAN Id
     * @param _encoderSernsorDir    Encoder Sensor Direction
     * @param _steerAnalogOffset    Steer Analog Offset in Degrees.
     */
    public SwerveData(String _name, int _driveCANId,  InvertedValue _driveInvertValue, int _steerCANId, InvertedValue _steerInvertValue, int _encoderCANId, SensorDirectionValue _encoderSernsorDir, double _steerAnalogOffset ){

        name = _name;
        driveCANId = _driveCANId;
        steerCANId = _steerCANId;
        encoderCANId = _encoderCANId;
        driveInvertValue = _driveInvertValue;
        steerInvertValue = _steerInvertValue;
        encoderDirection = _encoderSernsorDir;
        steerAngleOffset = _steerAnalogOffset;
    }

}
