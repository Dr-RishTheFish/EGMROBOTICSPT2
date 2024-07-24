package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class HoodOut extends CommandBase{
    private final HoodSubsystem m_hoodSubsystem;
    public double target_sensorUnits; 

    public HoodOut(HoodSubsystem hoodSubsystem, double target_position_m)
    {
        target_sensorUnits = target_position_m/Constants.RACK_PINION_CIRCUMFRENCE_METERS; 
        target_sensorUnits*= Constants.RACK_PINION_GEAR_RATIO;
        target_sensorUnits*=Constants.CLICKS;
        m_hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {

       //Values and Calculations
       SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.89107, 7.7967, 0.52962);
       double kf = feedforward.calculate(target_sensorUnits, 0); 
       kf = 0.1; 
       double cruiseVelocity_mps = 1; 
       double cruiseVelocity_clicks = (cruiseVelocity_mps/Constants.RACK_PINION_CIRCUMFRENCE_METERS)*Constants.RACK_PINION_GEAR_RATIO/10*Constants.CLICKS; 
       double cruiseAcceleration_mps = 20; 
       double cruiseAcceleration_clicks = cruiseVelocity_clicks * (cruiseAcceleration_mps/cruiseVelocity_mps); 

       //Config
       m_hoodSubsystem.hood_motor.configMotionCruiseVelocity(cruiseVelocity_clicks); 
       m_hoodSubsystem.hood_motor.configMotionAcceleration(cruiseAcceleration_clicks); 
       m_hoodSubsystem.hood_motor.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.ArbitraryFeedForward, kf);
    }

    /*@Override
    public void end(boolean interrupted) {
        m_hoodSubsystem.hood_motor.set(TalonFXControlMode.PercentOutput, 0);
    }*/

    public boolean isFinished(){   
       /* if(m_hoodSubsystem.hood_motor.getSelectedSensorPosition()>target_sensorUnits){
            return true;
        } 
        return false;*/
        return true; 
    }

}