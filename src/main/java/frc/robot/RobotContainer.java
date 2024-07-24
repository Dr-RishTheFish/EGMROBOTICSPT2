// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmExtensionCommandOut;
import frc.robot.commands.ArmExtensionCommandIn;
import frc.robot.commands.ArmRotationCommandUp;
import frc.robot.commands.ArmRotationCommandDown;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultRackandPinion;
import frc.robot.commands.DefaultShootCommand;
import frc.robot.commands.DefenseIntake;
import frc.robot.commands.RackPowerToZero;
import frc.robot.commands.ShootCommandToZero;
import frc.robot.commands.ReverseShooter;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.InputCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.commands.RackandPinionCommandOut;
import frc.robot.commands.RackandPinionCommandIn;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HoodOut;
import frc.robot.commands.HoodIn;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommandIn;
import frc.robot.commands.IntakeCommandOut;
import frc.robot.commands.VisionCommand;
import frc.robot.commands.checkArmSwingPos;
import frc.robot.commands.ArmExtensionCommandInandDown;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.RackandPinionSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.custom_buttons.DPADButton;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  // private final DashboardSubsystem m_dashboardSubsystem = new
  // DashboardSubsystem(m_drivetrainSubsystem,
  // m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem,
  // m_visionSubsystem);
  private final ArmExtensionSubsystem m_armExtensionSubsystem = new ArmExtensionSubsystem();
  private final RackandPinionSubsystem m_rackandPinionSubsystem = new RackandPinionSubsystem();
  private final ArmRotationSubsystem m_armRotationSubsystem = new ArmRotationSubsystem();
  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();

  // Instantiating Xbox controller
  private static final XboxController controllerOne = new XboxController(0);
  public static final XboxController controllerTwo = new XboxController(1);

  // Controller one buttons
  JoystickButton aButton = new JoystickButton(controllerOne, 1);
  JoystickButton bButton = new JoystickButton(controllerOne, 2);
  JoystickButton xButton = new JoystickButton(controllerOne, 3);
  JoystickButton yButton = new JoystickButton(controllerOne, 4);
  JoystickButton leftBumperButton = new JoystickButton(controllerOne, 5);
  JoystickButton rightBumperButton = new JoystickButton(controllerOne, 6);
  JoystickButton leftTinyButton = new JoystickButton(controllerOne, 7);
  JoystickButton rightTinyButton = new JoystickButton(controllerOne, 8);
  JoystickButton leftJoystickButton = new JoystickButton(controllerOne, 9);
  JoystickButton rightJoystickButton = new JoystickButton(controllerOne, 10);
  DPADButton upDPADButton = new DPADButton(controllerOne, 0);
  DPADButton downDPADButton = new DPADButton(controllerOne, 180);
  DPADButton rightDPADButton = new DPADButton(controllerOne, 90);
  DPADButton leftDPADButton = new DPADButton(controllerOne, 270);

  // Controller two buttons
  JoystickButton aButton2 = new JoystickButton(controllerTwo, 1);
  JoystickButton bButton2 = new JoystickButton(controllerTwo, 2);
  JoystickButton xButton2 = new JoystickButton(controllerTwo, 3);
  JoystickButton yButton2 = new JoystickButton(controllerTwo, 4);
  JoystickButton leftBumperButton2 = new JoystickButton(controllerTwo, 5);
  JoystickButton rightBumperButton2 = new JoystickButton(controllerTwo, 6);
  JoystickButton leftTinyButton2 = new JoystickButton(controllerTwo, 7);
  JoystickButton rightTinyButton2 = new JoystickButton(controllerTwo, 8);
  JoystickButton leftJoystickButton2 = new JoystickButton(controllerTwo, 9);
  JoystickButton rightJoystickButton2 = new JoystickButton(controllerTwo, 10);
  DPADButton upDPADButton2 = new DPADButton(controllerTwo, 0);
  DPADButton downDPADButton2 = new DPADButton(controllerTwo, 180);
  DPADButton rightDPADButton2 = new DPADButton(controllerTwo, 90);
  DPADButton leftDPADButton2 = new DPADButton(controllerTwo, 270);
  DPADButton downAndRight = new DPADButton(controllerTwo, 135);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    // Factory defaults
    m_shooterSubsystem.shooter_left.configFactoryDefault();
    m_intakeSubsystem.intake.configFactoryDefault();
    m_indexerSubsystem.indexer_left.configFactoryDefault();
    m_indexerSubsystem.indexer_right.configFactoryDefault();
    m_indexerSubsystem.indexer_right.configFactoryDefault();
    m_armRotationSubsystem.climber_rotation_motor.configFactoryDefault();
    m_rackandPinionSubsystem.rack_and_pinion.configFactoryDefault();
    m_hoodSubsystem.hood_motor.configFactoryDefault();
    m_armExtensionSubsystem.climber_wench_motor_right.configFactoryDefault();
    m_armExtensionSubsystem.climber_wench_motor_left.configFactoryDefault();
    m_armRotationSubsystem.climber_rotation_motor.configFactoryDefault();

    // Set in neutral mode
    m_shooterSubsystem.shooter_left.setNeutralMode(NeutralMode.Coast);
    m_intakeSubsystem.intake.setNeutralMode(NeutralMode.Coast);
    m_indexerSubsystem.indexer_left.setNeutralMode(NeutralMode.Coast);
    m_indexerSubsystem.indexer_right.setNeutralMode(NeutralMode.Coast);
    m_indexerSubsystem.indexer_right.setNeutralMode(NeutralMode.Coast);
    m_armRotationSubsystem.climber_rotation_motor.setNeutralMode(NeutralMode.Coast);
    m_rackandPinionSubsystem.rack_and_pinion.setNeutralMode(NeutralMode.Coast);
    m_hoodSubsystem.hood_motor.setNeutralMode(NeutralMode.Coast);
    m_armExtensionSubsystem.climber_wench_motor_right.setNeutralMode(NeutralMode.Coast);
    m_armExtensionSubsystem.climber_wench_motor_left.setNeutralMode(NeutralMode.Coast);
    m_armRotationSubsystem.climber_rotation_motor.setNeutralMode(NeutralMode.Brake);

    // Arm Rotation Set Up
    /*
     * m_armRotationSubsystem.climber_rotation_motor.setSelectedSensorPosition(0);
     * m_armRotationSubsystem.climber_rotation_motor.configForwardSoftLimitThreshold
     * (18000);
     * m_armRotationSubsystem.climber_rotation_motor.configForwardSoftLimitEnable(
     * true, 0);
     * m_armRotationSubsystem.climber_rotation_motor.configReverseSoftLimitThreshold
     * (1000);
     * m_armRotationSubsystem.climber_rotation_motor.configReverseSoftLimitEnable(
     * true, 0);
     */

    // Rack and Pinion Set Up
    m_rackandPinionSubsystem.rack_and_pinion.setSelectedSensorPosition(0);
    // m_rackandPinionSubsystem.rack_and_pinion.configForwardSoftLimitThreshold(46000);
    // m_rackandPinionSubsystem.rack_and_pinion.configForwardSoftLimitThreshold(44000);
    // m_rackandPinionSubsystem.rack_and_pinion.configForwardSoftLimitEnable(true,
    // 0);
    // m_rackandPinionSubsystem.rack_and_pinion.configReverseSoftLimitThreshold(4000);
    // m_rackandPinionSubsystem.rack_and_pinion.configReverseSoftLimitThreshold(12000);
    // m_rackandPinionSubsystem.rack_and_pinion.configReverseSoftLimitEnable(true,
    // 0);

    // PID Rack and Pinion
    m_rackandPinionSubsystem.rack_and_pinion.config_kF(0, 0, 30);
    m_rackandPinionSubsystem.rack_and_pinion.config_kP(0, 0.46885);
    m_rackandPinionSubsystem.rack_and_pinion.config_kI(0, 0);
    m_rackandPinionSubsystem.rack_and_pinion.config_kD(0, 0);
    m_rackandPinionSubsystem.rack_and_pinion.selectProfileSlot(0, Constants.RACK_AND_PINION_MOTOR);

    //Arm Set Up
    m_armExtensionSubsystem.climber_wench_motor_left.setSelectedSensorPosition(0);
    m_armExtensionSubsystem.climber_wench_motor_left.configForwardSoftLimitEnable(false,0);
    m_armExtensionSubsystem.climber_wench_motor_left.configReverseSoftLimitEnable(true,0);
    m_armExtensionSubsystem.climber_wench_motor_left.configReverseSoftLimitThreshold(-550000);

    m_armExtensionSubsystem.climber_wench_motor_right.setSelectedSensorPosition(0);
    m_armExtensionSubsystem.climber_wench_motor_right.configForwardSoftLimitEnable(false,0);
    m_armExtensionSubsystem.climber_wench_motor_right.configReverseSoftLimitEnable(true, 0);
    m_armExtensionSubsystem.climber_wench_motor_right.configReverseSoftLimitThreshold(-550000);

    // Hood Set Up
    m_hoodSubsystem.hood_motor.setInverted(true);
    m_hoodSubsystem.hood_motor.setSelectedSensorPosition(0);
    m_hoodSubsystem.hood_motor.configReverseSoftLimitEnable(true, 0);
    m_hoodSubsystem.hood_motor.configForwardSoftLimitEnable(true, 0);
    m_hoodSubsystem.hood_motor.configForwardSoftLimitThreshold(24000);
    m_hoodSubsystem.hood_motor.configReverseSoftLimitThreshold(0);
    m_hoodSubsystem.hood_motor.config_kF(0, 0, 30);
    m_hoodSubsystem.hood_motor.config_kP(0, 0.46885);
    m_hoodSubsystem.hood_motor.config_kI(0, 0);
    m_hoodSubsystem.hood_motor.config_kD(0, 0);
    m_hoodSubsystem.hood_motor.selectProfileSlot(0, Constants.RACK_AND_PINION_MOTOR);

    // Intake Set Up
    m_intakeSubsystem.intake.setInverted(true);
    double kP_I = .02;
    m_intakeSubsystem.intake.config_kF(0, .05, 30);
    m_intakeSubsystem.intake.config_kP(0, kP_I);
    m_intakeSubsystem.intake.config_kI(0, 0);
    m_intakeSubsystem.intake.config_kD(0, 0.7);
    m_intakeSubsystem.intake.selectProfileSlot(0, Constants.INTAKE_MOTER);

    // m_intakeSubsystem.intake.con

    // Shooter Set Up
    m_shooterSubsystem.shooter_left.setInverted(true);
    m_shooterSubsystem.shooter_right.setInverted(false);

    m_shooterSubsystem.shooter_right.config_kF(0, 0, 30);
    m_shooterSubsystem.shooter_right.config_kP(0, 0.02);
    m_shooterSubsystem.shooter_right.config_kI(0, 0);
    m_shooterSubsystem.shooter_right.config_kD(0, 5.0);
    m_shooterSubsystem.shooter_right.selectProfileSlot(0, Constants.SHOOTER_MOTER_RIGHT);

    m_shooterSubsystem.shooter_left.config_kF(0, 0, 30);
    m_shooterSubsystem.shooter_left.config_kP(0, .02);
    m_shooterSubsystem.shooter_left.config_kI(0, 0);
    m_shooterSubsystem.shooter_left.config_kD(0, 5.0);
    m_shooterSubsystem.shooter_left.selectProfileSlot(0, Constants.SHOOTER_MOTER_LEFT);

    //Drive train= set up

    // m_indexerSubsystem.indexer_right.configStatorCurrentLimit(new
    // StatorCurrentLimitConfiguration(true, 1, 1, 0.1));
    // m_shooterSubsystem.shooter_left.configStatorCurrentLimit(new
    // StatorCurrentLimitConfiguration(true, 5, 25, 0.1));
    m_rackandPinionSubsystem.rack_and_pinion
        .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 70, 0.1));
    // m_armRotationSubsystem.climber_rotation_motor.configStatorCurrentLimit(new
    // StatorCurrentLimitConfiguration(true,

    /* driver subsystem original 
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(controllerOne.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controllerOne.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controllerOne.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)); */


     m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(.5*controllerOne.getLeftX()+.5*Math.pow(controllerOne.getLeftX(),3)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(.5*controllerOne.getLeftY()+.5*Math.pow(controllerOne.getLeftY(),3)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(.5*controllerOne.getRightX()+.5*Math.pow(controllerOne.getRightX(),3)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_rackandPinionSubsystem.setDefaultCommand(new DefaultRackandPinion(m_rackandPinionSubsystem));
    m_shootSubsystem.setDefaultCommand(new DefaultShootCommand(m_shootSubsystem, m_indexerSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // controller one
    new Button(controllerOne::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // TODO: fix. Need button binding to fire command
    // new Button(m_controller::getAButton)
    // .whenHeld(m_shooterSubsystem::periodic);
    // Vision - Imports not configured
    // Changed URL
    HttpCamera limelightFeed = new HttpCamera("limelight-boss", "http://10.80.55.69:5800/stream.mjpg",
        HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(limelightFeed);
    rightBumperButton.whenHeld(new SequentialCommandGroup(
        new RackandPinionCommandOut(m_rackandPinionSubsystem, 0.52182), new IntakeCommandIn(m_intakeSubsystem)));
    leftDPADButton.whenHeld(new VisionCommand(m_visionSubsystem, 4));
    rightDPADButton.whenHeld(new VisionCommand(m_visionSubsystem, 2));
    // xButton.whenHeld(new RackandPinionCommandOut(m_rackandPinionSubsystem,
    // 0.52182));
    //aButton.whenPressed(new InstantCommand(m_shootSubsystem::toggleSpinOnIdle, m_shootSubsystem));
    //yButton.whenPressed(new DefenseIntake(m_rackandPinionSubsystem,0.015));
    yButton.whenPressed(new SequentialCommandGroup(new DefenseIntake(m_rackandPinionSubsystem,0.015), new WaitCommand(.75), new RackPowerToZero(m_rackandPinionSubsystem)));
    // xButton.whenPressed(new RackandPinionCommandIn(m_rackandPinionSubsystem,
    // 0.137));
    // yButton.whenPressed(new RackandPinionCommandSet(m_rackandPinionSubsystem,
    // 30000));
    // rightBumperButton.whenHeld(new IntakeCommandIn(m_intakeSubsystem));
    // leftBumperButton.whenHeld(new IntakeCommandOut(m_intakeSubsystem));
    // upDPADButton.whenPressed(new HoodOut(m_hoodSubsystem, 0.0508));
    // downDPADButton.whenHeld(new HoodIn(m_hoodSubsystem, 0.305));
    // aButton.whenPressed(new RackandPinionCommandOut(m_rackandPinionSubsystem,
    // 5000));
    // aButton.whenReleased(new RackandPinionCommandIn(m_rackandPinionSubsystem));

    // Controller two

    rightBumperButton2.whenHeld(new SequentialCommandGroup(new OutputCommand(m_indexerSubsystem).withTimeout(.25),
        new ShootCommand(m_shootSubsystem, m_intakeSubsystem), new InputCommand(m_indexerSubsystem)));
    rightBumperButton2.whenReleased(new ShootCommandToZero(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    aButton2.whenPressed(new InstantCommand(m_shootSubsystem::toggleSpinOnIdle, m_shootSubsystem));
    xButton2.whenPressed(new InstantCommand(m_visionSubsystem::toggleLed, m_visionSubsystem));
    // bButton2.whenHeld(new SetShooterSpeed(m_shooterSubsystem, 3000));
    // bButton2.whenReleased(new ShootCommandToZero(m_shooterSubsystem));
    // yButton2.whenHeld(new ReverseShooter(m_shooterSubsystem));
    leftBumperButton2.whenHeld(new ParallelCommandGroup(new RackandPinionCommandOut(m_rackandPinionSubsystem, 0.47102),
        new SequentialCommandGroup(new OutputCommand(m_indexerSubsystem).withTimeout(.25),
            new ShootCommand(m_shootSubsystem, m_intakeSubsystem), new InputCommand(m_indexerSubsystem))));
    leftBumperButton2.whenReleased(new ShootCommandToZero(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    //yButton2.whenHeld(new SequentialCommandGroup(new checkArmSwingPos(), new InstantCommand(m_visionSubsystem::toggleLed, m_visionSubsystem)));
    yButton2.whenHeld(new ArmExtensionCommandIn(m_armExtensionSubsystem)); 
    // rightBumperButton2.whenHeld(new InputCommand(m_indexerSubsystem));
    // leftBumperButton2.whenHeld(new OutputCommand(m_indexerSubsystem));
    //It was arm extension out ??
    downDPADButton2.whenHeld(new ArmExtensionCommandInandDown(m_armExtensionSubsystem, m_armRotationSubsystem));
    upDPADButton2.whenHeld(new ArmExtensionCommandIn(m_armExtensionSubsystem));
    rightDPADButton2.whenHeld(new ArmRotationCommandUp(m_armRotationSubsystem));
    leftDPADButton2.whenHeld(new ArmRotationCommandDown(m_armRotationSubsystem));
    // aButton2.whenHeld(new ArmsDownandRight(m_armRotationSubsystem,
    // m_armExtensionSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
        new OutputCommand(m_indexerSubsystem).withTimeout(.25),
        new ShootCommand(m_shootSubsystem, m_intakeSubsystem),
        new InputCommand(m_indexerSubsystem).withTimeout(1),
        new WaitCommand(3),
        new ShootCommandToZero(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem).withTimeout(1),
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> 0.0,
            () -> -1.5,
            () -> 0.0).withTimeout(1.5).andThen(
                new DefaultDriveCommand(
                    m_drivetrainSubsystem,
                    () -> 0.0,
                    () -> 0.0,
                    () -> 0.0)));
 
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;

  }
}
