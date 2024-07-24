package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class InputCommand extends CommandBase{
    private final IndexerSubsystem m_indexerSubsystem;

    public InputCommand(IndexerSubsystem indexerSubsystem)
    {
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    public void initialize() { 
        //System.out.println("InputCommand.execute()");       
        m_indexerSubsystem.indexer_left.set(TalonFXControlMode.PercentOutput, -.5);
        m_indexerSubsystem.indexer_right.set(TalonFXControlMode.PercentOutput, .40);
    }

    @Override
    public void execute() { 
        //System.out.println("InputCommand.execute()");       
        //m_indexerSubsystem.indexer_left.set(TalonFXControlMode.Velocity, -1000 * Constants.RPMtoEncoderConstant);
        //m_indexerSubsystem.indexer_right.set(TalonFXControlMode.Velocity, 1000 * Constants.RPMtoEncoderConstant);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
