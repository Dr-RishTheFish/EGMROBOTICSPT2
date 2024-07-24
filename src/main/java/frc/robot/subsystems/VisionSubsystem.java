package frc.robot.subsystems;  
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance; 
import frc.robot.subsystems.VisionSubsystem; 
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Constants; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.GenericHID; 
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; 
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; 
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;  
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import java.util.*; 
import java.util.Random;

public class VisionSubsystem extends SubsystemBase { 



	ShuffleboardTab tab = Shuffleboard.getTab("Limelight"); 
	static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-boss");
    public static NetworkTable visionSwitch = NetworkTableInstance.getDefault().getTable("limelight-boss");
    static NetworkTableEntry tx = table.getEntry("tx"); 
    static NetworkTableEntry ty = table.getEntry("ty"); 
    static NetworkTableEntry ta = table.getEntry("ta"); 
	static NetworkTableEntry ts = table.getEntry("ts");
	static NetworkTableEntry tv = table.getEntry("tv");

	static NetworkTableEntry ledMode = table.getEntry("ledMode");

	public static double pitchOffset = ty.getDouble(0);
	public static double yawOffset = tx.getDouble(0);
	public static double targetArea = ta.getDouble(0);
	public static double skew = ts.getDouble(0);
	
	private static boolean ledOn;
	
         
    public VisionSubsystem() { 
		ledOn = true;
	}
	
	@Override
	public void periodic(){
	
		// //post to smart dashboard periodically 
    	// /*tab.add("LimelightV", v); 
    	// tab.add("LimelightX", x); 
    	// tab.add("LimelightY", y); 
        // tab.add("LimelightArea", area);

  		pitchOffset = ty.getDouble(0);
  		yawOffset = tx.getDouble(0);
  		// targetArea = ta.getDouble(0);
		// skew = ts.getDouble(0);

		if (ledOn) {
			ledMode.setNumber(3);
		}
		else
		{
			ledMode.setNumber(1);
		}
	}

 
/*
  public double getPipline() {
	  
  }
  */
  /*public double getActivePipeline() {
	  return table.getEntry("getpipe").getDouble(0);
  }

  public void ledPipeline(){
	  table.getEntry("ledMode").setNumber(0);
  }

  public void setPipeline(int pipeline){
	  table.getEntry("pipeline").setNumber(pipeline);
  }

  public boolean getIsTargetFound() {
	double v = tv.getDouble(0);
	if (v == 0.0f){
		return false;
	}else {
		return true;
	}
	
}
  public double getTargetArea() {
	
	double a = ta.getDouble(0.0);
	return a;
}*/
  
public double getDistance() {
	return Constants.heightDifference / Math.tan(Math.toRadians(Constants.LimelightMountingAngle + ty.getDouble(0)) /12);
 
}

  
public double[] getShootingPower() {
	return new double[] {0,0,0};
}
  
public boolean toggleLed() {
	ledOn = !ledOn;
	return ledOn;
}
         
} 
	


