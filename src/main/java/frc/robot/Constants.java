// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import frc.robot.Lemonlight;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final int LEFT_SHOOTER_CAN_ID = 1;
    public final int RIGHT_SHOOTER_CAN_ID =  2;

    public final int FRONT_COLLECTOR_CAN_ID = 7 ;
    public final int BACK_CoLLECTOR_CAN_ID = 8;

    public final int LEFT_CLIMBER_CAN_ID = 9;
    public final int RIGHT_CLIMBER_CAN_ID = 10;
    //still need to assign CAN IDs
    public final int HIGH_CLIMBER_CAN_ID = 11;
    public final int ANGLE_CLIMBER_CAN_ID = 12;

    public final int LEFT_MASTER_CAN_ID = 1;
    public final int RIGHT_MASTER_CAN_ID = 2;
    public final int LEFT_FOLLOWER_CAN_ID = 3;
    public final int RIGHT_FOLLOWER_CAN_ID = 4;

    
    public  final double DRIVE_GEARING = 9.52; // need acutal value
    public  final double kTrackWidth = 0.5799666582; // meters 22.833333 inches 
    public  final double kWheelRadius = 0.0762; // meters 3 inches
    public  final int kEncoderResolution = 2048;

    
    public final double autoTurnSpeed = Math.PI;
    public final double autoLinearSpeed = 1;   

    public final double ksVolts = 0.9;
    public final double kvVoltSecondsPerMeter = 0.5;
    public final double kaVoltSecondsSquaredPerMeter = 5.8629E-05;
    public final double kVAngular = 0.1;
    public final double kAAngular = 0.1;

    //Shooter Constants
    public double SHOOTER_LEFT_KP = 0.065721; //3.1589E-09; //0.065721
    public double SHOOTER_RIGHT_KP =0.080915; //1.7041E-08;  //0.080915

    public double SHOOTER_LEFT_FEEDFORWARD = .00017;
    public double SHOOTER_RIGHT_FEEDFORWARD = .00017;
    public double kMaxVoltage = 12;
 
    

    public double manualShootSpeed = 600;

    //CAN ID (1) Shooter Feedforward
    public final double SHOOTER_LEFT_ksVolts = 0.056589;
    public final double SHOOTER_LEFT_kvVoltSecondsPerMeter = 0.066111;
    public final double SHOOTER_LEFT_kaVoltSecondsPerMeter = 0.0040136;

    //CAN ID (2) Shooter Feedforward

    public final double SHOOTER_RIGHT_ksVolts = 0.033183;
    public final double SHOOTER_RIGHT_kvVoltSecondsPerMeter = 0.065267;
    public final double SHOOTER_RIGHT_kaVoltSecondsPerMeter = 0.0078201;

    public final double kFlywheelGearing = 2;
    public final double kFlywheelMomentOfIntertia = .00032;

    



    public final double drive_kp = .023213;

    public NeutralMode driveMode = NeutralMode.Brake;

    public final double ENCODER_DISTANCE_PER_MARK = kWheelRadius * 2 * Math.PI / 2048;

    public final double maxVelocityMetersPerSecond = 3;
    public final double maxAccelerationMetersPerSecondSq = 1;

    public final double kRamseteB = 2.0;
    public final double kRamseteZeta = 0.7;

    public final double mountingAngle = -10;
    public final double mountedHeight = 26;
    public final double targetHeight = 104;
    public final double theta = 74;
    public final double heightOffset = targetHeight-mountedHeight;
    public final double staticDistance = 125;
    public final double readAngle = -3.09;
    public final double MaxRPM = 5700;

    public final double climbVoltage = -.4;
    public final double fallVoltage = .4;
    public final double coastVoltage = .15;

    public static class AutoConstants{
        public static final double maxVelocityMetersPerSecond = 3;
        public static final double maxAccelerationMetersPerSecondSq = 1;

        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
    }
    
}



