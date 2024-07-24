package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.commands.ArmRotationCommandUp;
import frc.robot.commands.ArmExtensionCommandIn;


public class ArmsDownandRight extends CommandBase{
    private final ArmRotationSubsystem m_armRotationSubsystem;
    private final ArmExtensionSubsystem m_armExtensionSubsystem;
    

    public ArmsDownandRight(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem)
    {
        m_armRotationSubsystem = armRotationSubsystem;
        m_armExtensionSubsystem = armExtensionSubsystem; 
        addRequirements(armRotationSubsystem);
        addRequirements(armExtensionSubsystem);

    }

    @Override
    public void execute() {
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, -.6);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, -.6);
        m_armRotationSubsystem.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, 1);


    
}

    @Override
    public void end(boolean interrupted) {
        m_armExtensionSubsystem.climber_wench_motor_left.set(TalonFXControlMode.PercentOutput, 0);
        m_armExtensionSubsystem.climber_wench_motor_right.set(TalonFXControlMode.PercentOutput, 0);
        m_armRotationSubsystem.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, 0);

    }

}
