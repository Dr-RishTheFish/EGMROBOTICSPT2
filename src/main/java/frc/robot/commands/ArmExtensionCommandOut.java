package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.InputCommand;
import frc.robot.commands.IntakeCommandIn;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class ArmExtensionCommandOut extends CommandBase{
    private final ArmExtensionSubsystem m_armExtensionSubsystem;

    public ArmExtensionCommandOut(ArmExtensionSubsystem armExtensionSubsystem)
    {
        m_armExtensionSubsystem = armExtensionSubsystem;
        addRequirements(armExtensionSubsystem);

    }

    @Override
    public void execute() {
       //m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.Velocity, 2000 * Constants.RPMtoEncoderConstant);
        //m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.Velocity, 2000 * Constants.RPMtoEncoderConstant);
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 1);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 1);
    }

    @Override
    public void end(boolean interrupted) {
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 0);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 0);

    }

}
