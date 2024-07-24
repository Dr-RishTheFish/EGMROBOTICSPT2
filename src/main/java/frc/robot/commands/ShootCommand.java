package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.DemandType;
import frc.robot.Constants;

public class ShootCommand extends CommandBase{

    double target_sensorUnits;
    double target_velocity_Clicksper100ms;

    ShootSubsystem m_shootSubsystem;
    IntakeSubsystem  m_intakeSubsystem; 


    public ShootCommand(ShootSubsystem shootSubsystem, IntakeSubsystem intakeSubsystem)
    {
        target_sensorUnits = 0.0; 
        target_velocity_Clicksper100ms = 0.0; 
        m_shootSubsystem = shootSubsystem; 
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_shootSubsystem, m_intakeSubsystem);

        }

    @Override
    public void initialize() {

        //set shooter values

        double shooterSpeed_RPM; 
        if(m_shootSubsystem.getSpinOnIdle()){
            shooterSpeed_RPM = m_shootSubsystem.shootSpeedTable.get(m_shootSubsystem.getClosestDistance_continuousShooter())[0];
        }
        else {
            shooterSpeed_RPM = m_shootSubsystem.shootSpeedTable.get(m_shootSubsystem.getClosestDistance())[0];
        }
 
        SmartDashboard.putNumber("Closest Distance", m_shootSubsystem.getClosestDistance());
        // Enable below to get from Shuffleboard
        //double shooterSpeed_RPM = m_shootSubsystem.shooterSpeed; 
        target_velocity_Clicksper100ms = m_shootSubsystem.convertRPMToClicks100ms(shooterSpeed_RPM);
        SimpleMotorFeedforward feedforward_shoot = new SimpleMotorFeedforward(0.59765, 0.10819, 0.0082958);
        double kf_shoot = feedforward_shoot.calculate(target_velocity_Clicksper100ms * 10 / 2048, 0); 
        //double nativeVelocity = (target_velocity_RPS * 2048 )/10;
        double nominalVoltage = 12.0; 
        SmartDashboard.putNumber("Velocity RPM", shooterSpeed_RPM);

        //configure motors
        m_shootSubsystem.shooter_right.set(TalonFXControlMode.Velocity, target_velocity_Clicksper100ms, DemandType.ArbitraryFeedForward, kf_shoot/nominalVoltage);
        m_shootSubsystem.shooter_left.set(TalonFXControlMode.Velocity, target_velocity_Clicksper100ms, DemandType.ArbitraryFeedForward, kf_shoot/nominalVoltage);
        
        //set hood values
        double hoodDistance_in = m_shootSubsystem.shootSpeedTable.get(m_shootSubsystem.getClosestDistance())[1]; 
        // Enable below to get from Shuffleboard
        //double hoodDistance_in = m_shootSubsystem.hoodDistance; 
        target_sensorUnits = m_shootSubsystem.convertInchesToClicks(hoodDistance_in); 
        SimpleMotorFeedforward feedforward_hood = new SimpleMotorFeedforward(0.89107, 7.7967, 0.52962);
        double kf_hood = feedforward_hood.calculate(target_sensorUnits, 0); 
        kf_hood = 0.1; 
        double cruiseVelocity_mps = 1; 
        double cruiseVelocity_clicks = (cruiseVelocity_mps/Constants.RACK_PINION_CIRCUMFRENCE_METERS)*Constants.RACK_PINION_GEAR_RATIO/10*Constants.CLICKS; 
        double cruiseAcceleration_mps = 20; 
        double cruiseAcceleration_clicks = cruiseVelocity_clicks * (cruiseAcceleration_mps/cruiseVelocity_mps); 
        SmartDashboard.putNumber("Hood Distance", hoodDistance_in);

        //Configure motors
        m_shootSubsystem.hood_motor.configMotionCruiseVelocity(cruiseVelocity_clicks); 
        m_shootSubsystem.hood_motor.configMotionAcceleration(cruiseAcceleration_clicks); 
        m_shootSubsystem.hood_motor.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.ArbitraryFeedForward, kf_hood);

        //Intake Set
        m_intakeSubsystem.intake.set(TalonFXControlMode.Velocity, -750 * Constants.RPMtoEncoderConstant);
    }

    @Override
    public void end(boolean interrupted) {
        /*m_shootSubsystem.shooter_right.set(TalonFXControlMode.PercentOutput, 0);
        m_shootSubsystem.shooter_left.set(TalonFXControlMode.PercentOutput, 0);
        m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, 0);*/

    }

    @Override
    public boolean isFinished(){
        double shooterError =  Math.abs(m_shootSubsystem.shooter_right.getSelectedSensorVelocity()-target_velocity_Clicksper100ms);
        double hoodError = Math.abs(target_sensorUnits-m_shootSubsystem.hood_motor.getSelectedSensorPosition());
        boolean isShootFinished = shooterError<250 && hoodError<600 ; 
        //SmartDashboard.putNumber("Shooter Error", shooterError);
        //SmartDashboard.putNumber("Current Shooter Sensor Velocity", m_shootSubsystem.shooter_right.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("Current RPM Velocity", m_shootSubsystem.convertClicksToRPM(m_shootSubsystem.shooter_right.getSelectedSensorVelocity()));
        //SmartDashboard.putNumber("Hood Error", hoodError);
        //SmartDashboard.putBoolean("Is Shoot Finished", isShootFinished);
        return isShootFinished;
    }

}
