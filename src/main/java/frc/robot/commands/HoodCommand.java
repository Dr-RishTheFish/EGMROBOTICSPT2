package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.HoodSubsystem;

public class HoodCommand extends CommandBase{
    private final HoodSubsystem m_hoodSubsystem;

    public HoodCommand(HoodSubsystem hoodSubsystem)
    {
        m_hoodSubsystem = hoodSubsystem;

        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        m_hoodSubsystem.hood_motor.set(TalonFXControlMode.PercentOutput, .3);
    }

    @Override
    public void end(boolean interrupted) {
        m_hoodSubsystem.hood_motor.set(TalonFXControlMode.PercentOutput, 0);
    }

}