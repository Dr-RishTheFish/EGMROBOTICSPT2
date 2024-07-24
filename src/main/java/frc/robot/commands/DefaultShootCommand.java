package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenix.motorcontrol.DemandType;
import frc.robot.Constants;

public class DefaultShootCommand extends CommandBase{

    double target_sensorUnits;
    double target_velocity_Clicksper100ms;

    ShootSubsystem m_shootSubsystem;
    IndexerSubsystem m_indexerSubsystem;  

    public DefaultShootCommand(ShootSubsystem shootSubsystem, IndexerSubsystem indexerSubsystem)
    {
        target_sensorUnits = 0.0; 
        target_velocity_Clicksper100ms = 0.0; 
        m_shootSubsystem = shootSubsystem;
        m_indexerSubsystem =  indexerSubsystem; 
        addRequirements(m_shootSubsystem,m_indexerSubsystem);
    }

    @Override
    public void initialize() {

        //set shooter values
        double shooterSpeed_RPM;
        double kf_shoot;
        double nominalVoltage;

        if (m_shootSubsystem .getSpinOnIdle()) {
            shooterSpeed_RPM = 3500; 
            target_velocity_Clicksper100ms = m_shootSubsystem.convertRPMToClicks100ms(shooterSpeed_RPM);
            SimpleMotorFeedforward feedforward_shoot = new SimpleMotorFeedforward(0.59765, 0.10819, 0.0082958);
            kf_shoot = feedforward_shoot.calculate(target_velocity_Clicksper100ms * 10 / 2048, 0); 
            nominalVoltage = 12.0;     

            // Shooter motors idle speed
            m_shootSubsystem.shooter_right.set(TalonFXControlMode.Velocity, target_velocity_Clicksper100ms, DemandType.ArbitraryFeedForward, kf_shoot/nominalVoltage);
            m_shootSubsystem.shooter_left.set(TalonFXControlMode.Velocity, target_velocity_Clicksper100ms, DemandType.ArbitraryFeedForward, kf_shoot/nominalVoltage);
            // Indexer motors idle speed
            m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, .2);
            m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, -.2);
        }
        else
        {
            // Shooter motors zero
            m_shootSubsystem.shooter_right.set(TalonFXControlMode.PercentOutput, 0);
            m_shootSubsystem.shooter_left.set(TalonFXControlMode.PercentOutput, 0);
            // Indexer motors zero
            m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, 0);
            m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, 0);
        }
        
        //set hood values
        double hoodDistance_in = 6.25; 
        target_sensorUnits = m_shootSubsystem.convertInchesToClicks(hoodDistance_in); 
        SimpleMotorFeedforward feedforward_hood = new SimpleMotorFeedforward(0.89107, 7.7967, 0.52962);
        double kf_hood = feedforward_hood.calculate(target_sensorUnits, 0); 
        kf_hood = 0.1; 
        double cruiseVelocity_mps = 1; 
        double cruiseVelocity_clicks = (cruiseVelocity_mps/Constants.RACK_PINION_CIRCUMFRENCE_METERS)*Constants.RACK_PINION_GEAR_RATIO/10*Constants.CLICKS; 
        double cruiseAcceleration_mps = 20; 
        double cruiseAcceleration_clicks = cruiseVelocity_clicks * (cruiseAcceleration_mps/cruiseVelocity_mps); 

        //Configure motors
        m_shootSubsystem.hood_motor.configMotionCruiseVelocity(cruiseVelocity_clicks); 
        m_shootSubsystem.hood_motor.configMotionAcceleration(cruiseAcceleration_clicks); 
        m_shootSubsystem.hood_motor.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.ArbitraryFeedForward, kf_hood);
    }

    @Override
    public void end(boolean interrupted) {
        m_shootSubsystem.shooter_right.set(TalonFXControlMode.PercentOutput, 0);
        m_shootSubsystem.shooter_left.set(TalonFXControlMode.PercentOutput, 0);
        m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, 0);
        m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, 0);

    }

}
