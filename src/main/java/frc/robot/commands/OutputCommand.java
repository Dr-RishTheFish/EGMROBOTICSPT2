package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class OutputCommand extends CommandBase {
    private final IndexerSubsystem m_indexerSubsystem;

    public OutputCommand(IndexerSubsystem indexerSubsystem) {
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, .3);
        m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, -.3);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, 0);
        m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, 0);
    }

    public boolean isFinished(){
        return false; 
    }

    
}
