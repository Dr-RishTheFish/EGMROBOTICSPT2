
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShooterSubsystem extends SubsystemBase {
    
    public TalonFX shooter_left = new TalonFX(Constants.SHOOTER_MOTER_LEFT, Constants.CANIVORE_NAME);
    public TalonFX shooter_right = new TalonFX(Constants.SHOOTER_MOTER_RIGHT, Constants.CANIVORE_NAME);

    public ShooterSubsystem() {

    
    }
    
}