package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;



public class ArmExtensionCommandInandDown extends CommandBase{
    private final ArmExtensionSubsystem m_armExtensionSubsystem;
    private final ArmRotationSubsystem m_armRotationSubsystem;

    public ArmExtensionCommandInandDown(ArmExtensionSubsystem armExtensionSubsystem, ArmRotationSubsystem armRotationSubsystem)
    {
        m_armExtensionSubsystem = armExtensionSubsystem;
        m_armRotationSubsystem = armRotationSubsystem; 
        addRequirements(armExtensionSubsystem, armRotationSubsystem);

    }

    @Override
    public void execute() {
        //m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.Velocity, -2000 * Constants.RPMtoEncoderConstant);
        //m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.Velocity, -2000 * Constants.RPMtoEncoderConstant);
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 1);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 1);
        m_armRotationSubsystem.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, -.1);

    }

    @Override
    public void end(boolean interrupted) {
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 0);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 0);
        m_armRotationSubsystem.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, 0);

    }

}
