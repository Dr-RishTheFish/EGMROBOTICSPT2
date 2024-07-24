
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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
import frc.robot.commands.InputCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.subsystems.IndexerSubsystem;
import java.util.*;

public class IndexerSubsystem extends SubsystemBase{
    public TalonFX indexer_left = new TalonFX(Constants.INDEXER_LEFT_MOTOR, Constants.CANIVORE_NAME);
    public TalonFX indexer_right = new TalonFX(Constants.INDEXER_RIGHT_MOTOR, Constants.CANIVORE_NAME);

    public IndexerSubsystem(){
        //indexer_right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 10, 0.1));

    }

    public void periodic() {
       
    } 
    
}
