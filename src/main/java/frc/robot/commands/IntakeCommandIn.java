package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandIn extends CommandBase{
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeCommandIn(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, .2);
        m_intakeSubsystem.intake.set(TalonFXControlMode.Velocity, -2000 * Constants.RPMtoEncoderConstant);
        //m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, .3);

    }

    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, 0);
    }
}
