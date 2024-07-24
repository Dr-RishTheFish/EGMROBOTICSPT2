package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance; 
import frc.robot.subsystems.VisionSubsystem.*;

public class SetShooterSpeed extends CommandBase{
    private final ShooterSubsystem m_shooterSubsystem;
    public double target_velocity_RPS; 

    public SetShooterSpeed(ShooterSubsystem shooterSubsystem, double target_velocity_RPM)
    {
        m_shooterSubsystem = shooterSubsystem;
        target_velocity_RPS = target_velocity_RPM/60; 
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute() {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.57424, 0.10548, 0.018368);
        double kf = feedforward.calculate(target_velocity_RPS, 0); 
        double nativeVelocity = (target_velocity_RPS * 2048 )/10;
        double nominalVoltage = 12.0; 
        m_shooterSubsystem.shooter_right.set(TalonFXControlMode.Velocity, nativeVelocity, DemandType.ArbitraryFeedForward, kf/nominalVoltage);
        m_shooterSubsystem.shooter_left.set(TalonFXControlMode.Velocity, nativeVelocity, DemandType.ArbitraryFeedForward, kf/nominalVoltage);

    }

    /*@Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.shooter_right.set(TalonFXControlMode.PercentOutput, 0);
    }*/

    public boolean isFinished(){   
        if(m_shooterSubsystem.shooter_right.getSelectedSensorVelocity()>target_velocity_RPS){
            return true;
        } 
        return false;
    }

   

 
 
}

