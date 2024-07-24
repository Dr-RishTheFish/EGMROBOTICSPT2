package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandOut extends CommandBase{
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeCommandOut(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.intake.set(TalonFXControlMode.Velocity, 500 * Constants.RPMtoEncoderConstant);

    }

    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake.set(TalonFXControlMode.PercentOutput, 0);
    }
}
