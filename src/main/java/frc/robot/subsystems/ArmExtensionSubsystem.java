
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmExtensionSubsystem extends SubsystemBase {
    public TalonFX climber_wench_motor_left = new TalonFX(Constants.CLIMBER_WENCH_MOTOR_LEFT, Constants.CANIVORE_NAME);
    public TalonFX climber_wench_motor_right = new TalonFX(Constants.CLIMBER_WENCH_MOTOR_RIGHT, Constants.CANIVORE_NAME);
    

    public ArmExtensionSubsystem() {

        climber_wench_motor_left.follow(climber_wench_motor_right);
    }

    @Override
    public void periodic() {
       
    }
}