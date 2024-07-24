package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooter extends CommandBase{
    private final ShooterSubsystem m_shooterSubsystem;

    public ReverseShooter(ShooterSubsystem shooterSubsystem)
    {
        m_shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.shooter_right.set(TalonFXControlMode.Velocity, -1000 * Constants.RPMtoEncoderConstant);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.shooter_right.set(TalonFXControlMode.PercentOutput, 0);
    }

}