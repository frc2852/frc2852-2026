// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

public static class HoodConstants {
    
  //PID Constants (slot 0)
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  //Motion Magic Constants
  public static final double Motion_Magic_Crusive_Velocity = 360.0; //degrees
  public static final double Motion_Magic_Acceleration = 360.0; //degrees
  public static final double Motion_Magic_Jerk = 0.0; //degrees

  //Soft Limits
  public static final double Forward_Soft_Limit_Threshhold = 360.0; //degrees
  public static final double Reverse_Soft_Limit_Threshhold = 360.0; //degrees

  //Curet Limits
  public static final double Stator_Current_Limit = 1.0; //amps
  public static final double Supply_Current_Limit = 1; //amps
  public static final double Supply_Current_Lower_Limit = 1.0; //amps
  public static final double Supply_Current_Lower_Time = 1.0; //seconds

  //Torque Curent Limits
  public static final double Peak_Forward_Torque_Current = 1.0; //Nm
  public static final double Peak_Reverse_Torque_Current = 1.0; //Nm

  //Hood Positions
  public static final double Hood_Min_Position = 0.0; //degrees
  public static final double Hood_Max_Position = 90.0; //degrees
  public static final double Hood_Position_Tolerance = 1.0; //degrees

  //Gear Ratio
  public static final double Hood_Gear_Ratio = 1.0; //ratio

  //CANcoder Offset
  public static final double Magnet_Offset = 0.0; //degrees

  //Sensor
  public static final double Feedback_Remote_Sensor_ID = 25;
  public static final double Rotor_To_Sensor_Ratio = 1.0;
  
  }
   
}
