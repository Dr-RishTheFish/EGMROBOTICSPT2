// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //ShootSubsystem shoot = new ShootSubsystem(); 
    //SmartDashboard.putNumber("Current RPM Velocity", (shoot.convertClicksToRPM(shoot.shooter_right.getSelectedSensorVelocity())));
    //SmartDashboard.putNumber("Velocity Error", (shoot.convertClicksToRPM(shoot.convertClicksToRPM(shoot.shooter_right.getClosedLoopError()))));

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LiveWindow.setEnabled(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*Pigeon2 m_pigeon = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, Constants.CANIVORE_NAME); 
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
    NetworkTable table = inst.getTable("datatable");
    NetworkTableEntry yawEntry = table.getEntry("yaw");
    NetworkTableEntry pitchEntry = table.getEntry("pitch");
    NetworkTableEntry rollEntry = table.getEntry("roll");

    NetworkTableEntry yawAccEntry = table.getEntry("yaw acceleration");
    NetworkTableEntry pitchAccEntry = table.getEntry("pitch acceleration");
    NetworkTableEntry rollAccEntry = table.getEntry("roll acceleration");

    NetworkTableEntry yawGyroEntry = table.getEntry("yaw gyro");
    NetworkTableEntry pitchGyroEntry = table.getEntry("pitch gyro");
    NetworkTableEntry rollGyroEntry = table.getEntry("roll gyro");

    double[] ypr = new double[3];
    short[] acceleration_values = new short[3];
    double[] gyro = new double[3]; 

    m_pigeon.getYawPitchRoll(ypr);
    m_pigeon.getBiasedAccelerometer(acceleration_values);
    m_pigeon.getAccumGyro(gyro); 

    yawEntry.setDouble(ypr[0]);
    pitchEntry.setDouble(ypr[1]);
    rollEntry.setDouble(ypr[2]);

    yawAccEntry.setDouble(acceleration_values[0]); 
    pitchAccEntry.setDouble(acceleration_values[1]);
    rollAccEntry.setDouble(acceleration_values[2]); 

    yawGyroEntry.setDouble(gyro[0]);
    pitchGyroEntry.setDouble(gyro[1]);
    rollGyroEntry.setDouble(gyro[2]); 
        
    System.out.println("Pigeon Yaw is: " + ypr[0]);
    System.out.println("Pigeon Yaw Acceleration are: " + acceleration_values[0]); */

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
