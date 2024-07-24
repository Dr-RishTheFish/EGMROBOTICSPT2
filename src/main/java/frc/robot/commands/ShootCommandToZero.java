package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommandToZero extends CommandBase{
    private final ShooterSubsystem m_shooterSubsystem;
    private final IndexerSubsystem m_indexerSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public ShootCommandToZero(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem)
    {
        m_shooterSubsystem = shooterSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, 0);
    }
}