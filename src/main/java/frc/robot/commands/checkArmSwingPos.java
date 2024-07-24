package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class checkArmSwingPos extends CommandBase {

    double gyro2 = 0;
    double gyro1 = 0;
    double gyro0 = 0;

    Pigeon2 m_pigeon = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, Constants.CANIVORE_NAME);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    NetworkTableEntry yawEntry = table.getEntry("yaw");
    NetworkTableEntry pitchEntry = table.getEntry("pitch");
    NetworkTableEntry rollEntry = table.getEntry("roll");

    NetworkTableEntry yawAccEntry = table.getEntry("yaw acceleration");
    NetworkTableEntry pitchAccEntry = table.getEntry("pitch acceleration");
    NetworkTableEntry rollAccEntry = table.getEntry("roll acceleration");

    NetworkTableEntry yawGyroEntry = table.getEntry("yaw gyro");
    NetworkTableEntry pitchGyroEntry = table.getEntry("pitch gyro");
    NetworkTableEntry rollGyroEntry = table.getEntry("roll gyro");

    double[] ypr = new double[3];
    short[] acceleration_values = new short[3];
    double[] gyro = new double[3]; 

    public void CommandBase(){
     
    /*m_pigeon.getYawPitchRoll(ypr);
    m_pigeon.getBiasedAccelerometer(acceleration_values);
    m_pigeon.getAccumGyro(gyro); 

    yawEntry.setDouble(ypr[0]);
    pitchEntry.setDouble(ypr[1]);
    rollEntry.setDouble(ypr[2]);

    yawAccEntry.setDouble(acceleration_values[0]); 
    pitchAccEntry.setDouble(acceleration_values[1]);
    rollAccEntry.setDouble(acceleration_values[2]); 

    yawGyroEntry.setDouble(gyro[0]);
    pitchGyroEntry.setDouble(gyro[1]);
    rollGyroEntry.setDouble(gyro[2]);*/

    }

    public void execute(){
        gyro2 = gyro1;
        gyro1 = gyro0;
        m_pigeon.getYawPitchRoll(ypr); 
        gyro0 = ypr[0];

    }

    public boolean isFinished(){
        return gyro0>gyro1; 
    }
}
