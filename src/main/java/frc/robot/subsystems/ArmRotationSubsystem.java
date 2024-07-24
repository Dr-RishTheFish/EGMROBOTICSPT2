
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmRotationSubsystem extends SubsystemBase {
    public TalonFX climber_rotation_motor = new TalonFX(Constants.CLIMBER_ROTATION_MOTOR, Constants.CANIVORE_NAME);

    public ArmRotationSubsystem() {

        /*climber_rotation_motor.setInverted(true);
        climber_rotation_motor.configForwardSoftLimitThreshold(18000); 
        climber_rotation_motor.configForwardSoftLimitEnable(true, 0);
        climber_rotation_motor.configReverseSoftLimitThreshold(1000);
        climber_rotation_motor.configReverseSoftLimitEnable(true, 0);*/

    }

    @Override
    public void periodic() {
       
    }
}