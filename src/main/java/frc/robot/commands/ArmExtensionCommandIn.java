package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.ArmExtensionSubsystem;


public class ArmExtensionCommandIn extends CommandBase{
    private final ArmExtensionSubsystem m_armExtensionSubsystem;

    public ArmExtensionCommandIn(ArmExtensionSubsystem armExtensionSubsystem)
    {
        m_armExtensionSubsystem = armExtensionSubsystem;
        addRequirements(armExtensionSubsystem);

    }

    @Override
    public void execute() {
        //m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.Velocity, -2000 * Constants.RPMtoEncoderConstant);
        //m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.Velocity, -2000 * Constants.RPMtoEncoderConstant);
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, -1);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, -1);

    }

    @Override
    public void end(boolean interrupted) {
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 0);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 0);

    }

}
