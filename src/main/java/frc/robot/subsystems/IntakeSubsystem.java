
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommandIn;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; 
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.*;


public class IntakeSubsystem extends SubsystemBase {
    public TalonFX intake = new TalonFX(Constants.INTAKE_MOTER, Constants.CANIVORE_NAME);

    public IntakeSubsystem() {
        intake.setInverted(true);
        double kP_I = .02; 
        intake.config_kF(0, .05, 30); 
        intake.config_kP(0, kP_I); 
        intake.config_kI(0, 0);            
        intake.config_kD(0, 0.7);
        intake.selectProfileSlot(0, Constants.INTAKE_MOTER);
    }

    @Override
    public void periodic() {
       
    }

    
}