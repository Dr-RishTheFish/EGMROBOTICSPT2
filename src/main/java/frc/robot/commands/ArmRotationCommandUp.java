package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotationSubsystem;


public class ArmRotationCommandUp extends CommandBase{
    private final ArmRotationSubsystem m_armRotationCommand;

    public ArmRotationCommandUp(ArmRotationSubsystem armRotationCommandSubsystem)
    {
        m_armRotationCommand = armRotationCommandSubsystem;
        addRequirements(armRotationCommandSubsystem);

    }

    @Override
    public void execute() {
//        m_armRotationCommand.climber_rotation_motor.set(TalonFXControlMode.Velocity, 2000 * Constants.RPMtoEncoderConstant);
        m_armRotationCommand.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, .4);
    }

    @Override
    public void end(boolean interrupted) {
        m_armRotationCommand.climber_rotation_motor.set(TalonFXControlMode.PercentOutput, 0);

    }

}
