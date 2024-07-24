package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.RackandPinionSubsystem;
             
public class RackandPinionCommandSet extends CommandBase{
    private final RackandPinionSubsystem m_rackandPinionSubsystem;
    private int m_setPoint;

    public RackandPinionCommandSet(RackandPinionSubsystem rackandPinionSubsystem, int SetPoint)
    {
        m_rackandPinionSubsystem = rackandPinionSubsystem;
        m_setPoint = SetPoint;

        addRequirements(rackandPinionSubsystem);
    }

    @Override
    public void execute() {
        m_rackandPinionSubsystem.rack_and_pinion.set(TalonFXControlMode.PercentOutput, -.75);
        if(this.isFinished()){
            this.end(true);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_rackandPinionSubsystem.rack_and_pinion.set(TalonFXControlMode.PercentOutput, 0);
    }

    public boolean isFinished(){
        if(m_rackandPinionSubsystem.rack_and_pinion.getSelectedSensorPosition() < m_setPoint) return true;
        return false; 
    }

}