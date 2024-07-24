// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String CANIVORE_NAME = "BossBus";
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .5969;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5969;

    public static final int DRIVETRAIN_PIGEON_ID = 13;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(190.1); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 8;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(332.0); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(166.8); // FIXME Measure and set back left steer offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(44.21); // FIXME Measure and set back right steer offset

    // Shooter
    public static final double maxMotorRPM = 6000.0;
    public static final double RPMtoEncoderConstant = 4096.0/600.0;

    public static final int SHOOTER_MOTER_LEFT = 41;
    public static final int SHOOTER_MOTER_RIGHT = 42;
    public static final int HOOD_MOTOR = 43; 

    public static final int CLIMBER_WENCH_MOTOR_LEFT = 51;
    public static final int CLIMBER_WENCH_MOTOR_RIGHT = 52;
    public static final int CLIMBER_ROTATION_MOTOR = 53; 

    public static final int INTAKE_MOTER = 21;

    public static final int INDEXER_LEFT_MOTOR = 31;
    public static final int INDEXER_RIGHT_MOTOR = 32;

    public static final int RACK_AND_PINION_MOTOR = 22;
    
    //public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-boss");
	  //public static NetworkTableEntry ty = table.getEntry("ty");
		//public static double pitchOffset = ty.getDouble(0);
		//public static double targetOffsetAngle_Vertical = ty.getDouble(0.0);
		public static double LimelightMountingAngle = 25.25;
		public static double limelightLensHeightInches = 34.0;
		public static double goalHeightInches = 104.0; //change back to orignal 104 
		//public static double angleToGoalDegrees = LimelightMountingAngle + targetOffsetAngle_Vertical;
		//public static double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
		public static double heightDifference = goalHeightInches - limelightLensHeightInches;
		//public static double distanceFromLimelightToGoalInches = heightDifference/Math.tan(angleToGoalRadians);
    

    //Other Constants
    public static final int CLICKS = 2048;
    public static final double RACK_PINION_CIRCUMFRENCE_METERS = 0.0797964534;
    public static final double RACK_PINION_GEAR_RATIO = 5.625;

    public static final double HOOD_CIRCUMFRENCE = 0.0797964534; 
    public static final double HOOD_GEAR_RATIO = 3.75; 
}
