
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class HoodSubsystem extends SubsystemBase {
    public TalonFX hood_motor = new TalonFX(Constants.HOOD_MOTOR, Constants.CANIVORE_NAME);

    public HoodSubsystem() {

    }

    @Override
    public void periodic() {
       
    }
}