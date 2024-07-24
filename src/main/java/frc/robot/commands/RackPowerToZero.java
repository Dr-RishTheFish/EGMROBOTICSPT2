package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RackandPinionSubsystem;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class RackPowerToZero extends CommandBase{
    private final RackandPinionSubsystem m_rackandPinionSubsystem;

    public RackPowerToZero(RackandPinionSubsystem rackandPinionSubsystem)
    {
        m_rackandPinionSubsystem = rackandPinionSubsystem;
        addRequirements(rackandPinionSubsystem);
    }

    @Override
    public void execute() {
        m_rackandPinionSubsystem.rack_and_pinion.set(TalonFXControlMode.PercentOutput, 0);
    }
    
}
