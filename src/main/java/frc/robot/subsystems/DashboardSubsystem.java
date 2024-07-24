package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class DashboardSubsystem extends SubsystemBase{
    
    private final DrivetrainSubsystem _drivetrainSubsystem;
    private final ShooterSubsystem _shooterSubsystem;
    private final IntakeSubsystem _intakeSubsystem;
    private final IndexerSubsystem _indexerSubsystem;
    private final VisionSubsystem _visionSubsystem;
 
    //private final ShooterSubsystem m_shooterSubsystem;
    //private final IntakeSubsystem m_intakeSubsystem;
    // private final IndexerSubsystem m_indexerSubsystem;
    // private final DrivetrainSubsystem m_DrivetrainSubsystem;


    public DashboardSubsystem(DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, 
    IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, VisionSubsystem visionSubsystem)
    {
        _shooterSubsystem = shooterSubsystem;
        _indexerSubsystem = indexerSubsystem;
        _intakeSubsystem = intakeSubsystem;
        _drivetrainSubsystem = drivetrainSubsystem;
        _visionSubsystem = visionSubsystem;
 
        
    }

    @Override
    public void periodic() {
            //SmartDashboard.putNumber("Shooter", _shooterSubsystem.shooter_left.getSelectedSensorVelocity());
            //System.out.println(_shooterSubsystem.shooter_left.getSelectedSensorVelocity());
            //SmartDashboard.putNumber("Shooter", _shooterSubsystem.shooter_left.getActiveTrajectoryVelocity());
            //System.out.println(_shooterSubsystem.shooter_left.getActiveTrajectoryVelocity());
            
            
            /*SmartDashboard.putNumber("Sensor Velocity: ", _shooterSubsystem.shooter.getSelectedSensorVelocity());
            SmartDashboard.putNumber("Sensor Velocity: ", _shooterSubsystem.shooter.getTemperature());*/

            //SmartDashboard.updateValues();
                
    }

}
 
 

