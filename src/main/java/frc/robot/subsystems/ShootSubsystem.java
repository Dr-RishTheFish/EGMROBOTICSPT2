
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTable; 
import edu.wpi.first.networktables.NetworkTableEntry; 
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class ShootSubsystem extends SubsystemBase {

    private boolean spinOnIdle = false;
    
    public TalonFX shooter_left = new TalonFX(Constants.SHOOTER_MOTER_LEFT, Constants.CANIVORE_NAME);
    public TalonFX shooter_right = new TalonFX(Constants.SHOOTER_MOTER_RIGHT, Constants.CANIVORE_NAME);
    public TalonFX hood_motor = new TalonFX(Constants.HOOD_MOTOR, Constants.CANIVORE_NAME);

    SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(); 
    public HashMap<Double, Double[]> shootSpeedTable = new HashMap<Double,Double[]>();
    public HashMap<Double, Double[]> shootSpeedTable_continuousShooter = new HashMap<Double,Double[]>();

	static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-boss"); 
    static NetworkTableEntry tx = table.getEntry("tx"); 
    static NetworkTableEntry ty = table.getEntry("ty"); 
    static NetworkTableEntry ta = table.getEntry("ta"); 
	static NetworkTableEntry ts = table.getEntry("ts");
	static NetworkTableEntry tv = table.getEntry("tv");

	public static double verticalOffset = ty.getDouble(0);
	public static double horizontalOffset = tx.getDouble(0);
	public static double targetArea = ta.getDouble(0);
	public static double skew = ts.getDouble(0);

    private ShuffleboardTab tab = Shuffleboard.getTab("Shoot System");

    //Enable to get inputs from shuffleboard
    public NetworkTableEntry hoodDistanceEntry = tab.add("Hood Distance", 1).getEntry();
    public NetworkTableEntry shooterSpeedEntry = tab.add("Shooter Speed", 1).getEntry();

    public double hoodDistance;
    public double shooterSpeed;

    public ShootSubsystem() {

        //Hood Set Up 
        hood_motor.setInverted(true);
        hood_motor.setSelectedSensorPosition(0);
        hood_motor.configReverseSoftLimitEnable(true, 0);
        hood_motor.configForwardSoftLimitEnable(true, 0);
        hood_motor.configForwardSoftLimitThreshold(24000); 
        hood_motor.configReverseSoftLimitThreshold(0); 
        hood_motor.config_kF(0, 0, 30); 
        hood_motor.config_kP(0, 0.46885); 
        hood_motor.config_kI(0, 0);            
        hood_motor.config_kD(0, 0);
        hood_motor.selectProfileSlot(0, Constants.RACK_AND_PINION_MOTOR);

    
    //Shooter Set Up 
        shooter_left.setInverted(true);
        shooter_right.setInverted(false);

        shooter_right.config_kF(0, 0, 30); 
        shooter_right.config_kP(0, 0.02); 
        shooter_right.config_kI(0, 0);
        shooter_right.config_kD(0, 5.0); 
        shooter_right.selectProfileSlot(0, Constants.SHOOTER_MOTER_RIGHT);

        shooter_left.config_kF(0, 0, 30); 
        shooter_left.config_kP(0, .02); 
        shooter_left.config_kI(0, 0);
        shooter_left.config_kD(0, 5.0); 
        shooter_left.selectProfileSlot(0, Constants.SHOOTER_MOTER_LEFT);
    
        //speed [0]
        //hood distance [1]
        //speed table on no continuous shooter
        shootSpeedTable.put(50.0, new Double[]{3250.0, 3.5});
        shootSpeedTable.put(68.0, new Double[]{3250.0, 4.5});
        shootSpeedTable.put(83.0, new Double[]{3250.0, 4.85});
        shootSpeedTable.put(93.0, new Double[]{3250.0, 5.0});
        shootSpeedTable.put(105.0, new Double[]{3500.0, 6.0});
        shootSpeedTable.put(117.0, new Double[]{3500.0, 6.25});
        shootSpeedTable.put(130.0, new Double[]{3550.0, 6.25});
        shootSpeedTable.put(143.0, new Double[]{3600.0, 6.5});
        shootSpeedTable.put(154.0, new Double[]{3650.0, 6.75});
        shootSpeedTable.put(167.0, new Double[]{3700.0, 7.25});
        shootSpeedTable.put(179.0, new Double[]{3750.0, 7.5});
        shootSpeedTable.put(193.0, new Double[]{4000.0, 7.25});
        shootSpeedTable.put(209.0, new Double[]{4300.0, 8.0});
        shootSpeedTable.put(221.0, new Double[]{4300.0, 8.0});
        shootSpeedTable.put(236.0, new Double[]{4350.0, 8.5});
        shootSpeedTable.put(252.0, new Double[]{4300.0, 8.75});
        shootSpeedTable.put(278.0, new Double[]{4400.0, 9.25});
        shootSpeedTable.put(285.0, new Double[]{4400.0, 9.5});
        shootSpeedTable.put(307.0, new Double[]{4600.0, 10.0});
        shootSpeedTable.put(327.0, new Double[]{4750.0, 9.75});

        //speed table on continuous shooter
        shootSpeedTable_continuousShooter.put(50.0, new Double[]{3400.0, 3.5});
        shootSpeedTable_continuousShooter.put(68.0, new Double[]{3400.0, 4.5});
        shootSpeedTable_continuousShooter.put(83.0, new Double[]{3400.0, 4.75});
        shootSpeedTable_continuousShooter.put(93.0, new Double[]{3400.0, 5.0});
        shootSpeedTable_continuousShooter.put(105.0, new Double[]{3650.0, 6.0});
        shootSpeedTable_continuousShooter.put(117.0, new Double[]{3650.0, 6.25});
        shootSpeedTable_continuousShooter.put(130.0, new Double[]{3700.0, 6.25});
        shootSpeedTable_continuousShooter.put(143.0, new Double[]{3750.0, 6.5});
        shootSpeedTable_continuousShooter.put(154.0, new Double[]{3800.0, 6.75});
        shootSpeedTable_continuousShooter.put(167.0, new Double[]{3850.0, 7.25});
        shootSpeedTable_continuousShooter.put(179.0, new Double[]{3900.0, 7.5});
        shootSpeedTable_continuousShooter.put(193.0, new Double[]{4150.0, 7.25});
        shootSpeedTable_continuousShooter.put(209.0, new Double[]{4450.0, 8.0});
        shootSpeedTable_continuousShooter.put(221.0, new Double[]{4450.0, 8.0});
        shootSpeedTable_continuousShooter.put(236.0, new Double[]{4500.0, 8.5});
        shootSpeedTable_continuousShooter.put(252.0, new Double[]{4450.0, 8.75});
        shootSpeedTable_continuousShooter.put(278.0, new Double[]{4550.0, 9.25});
        shootSpeedTable_continuousShooter.put(285.0, new Double[]{4550.0, 9.5});
        shootSpeedTable_continuousShooter.put(307.0, new Double[]{4850.0, 10.0});
        shootSpeedTable_continuousShooter.put(327.0, new Double[]{5000.0, 9.75});

  }

    @Override
    public void periodic() {
        verticalOffset = table.getEntry("ty").getDouble(0);
  		horizontalOffset = tx.getDouble(0);
        //System.out.println("Vertical Offset: "+ verticalOffset);
        //System.out.println("Horizontal Offset: "+ horizontalOffset);
        
        SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
        SmartDashboard.putNumber("Vertical Offset", verticalOffset);
        SmartDashboard.putNumber("Table Velocity", shootSpeedTable.get(getClosestDistance())[0]); 
        SmartDashboard.putNumber("Table Hood Position", convertToInches(shootSpeedTable.get(getClosestDistance())[1])); 
        SmartDashboard.putNumber("Conversion", convertInchesToClicks(4)); 
        SmartDashboard.putNumber("Distance", getDistance());

        SmartDashboard.putNumber("Shooter Talon Velocity (RPM)", convertClicksToRPM(shooter_right.getClosedLoopTarget()));
        SmartDashboard.putNumber("Shooter Talon Error (RPM)", convertClicksToRPM(shooter_right.getClosedLoopError()));
        
        //Enable to get inputs from shuffleboard
        //hoodDistance = hoodDistanceEntry.getDouble(1.0);
        //shooterSpeed = shooterSpeedEntry.getDouble(1.0);

        /*System.out.println("Shooter Error: " + Math.abs(shooter_right.getSelectedSensorVelocity()-convertRPMToClicks(2000)));
        System.out.println("Hood Error: " + Math.abs(convertInchesToClicks(4.0)-hood_motor.getSelectedSensorPosition()));
        System.out.println("Hood Position: "+hood_motor.getSelectedSensorPosition()); */
    }

    public boolean toggleSpinOnIdle() {
        spinOnIdle = !spinOnIdle;
        
        return spinOnIdle;
    }

    public boolean getSpinOnIdle() {
        return spinOnIdle;
    }

    public double getClosestDistance() {
        double distance = this.getDistance(); 
        double tempError = 0;
        double minAbsError = Double.MAX_VALUE;
        double distanceKey = 0;
        for(Map.Entry<Double,Double[]> speedEntry : shootSpeedTable.entrySet()) {
            tempError = Math.abs(speedEntry.getKey() - distance);

            if (tempError < minAbsError) {
                minAbsError = tempError;
                distanceKey = speedEntry.getKey();
            }
        }

        return distanceKey;
    }

    public double getClosestDistance_continuousShooter() {
        double distance = this.getDistance(); 
        double tempError = 0;
        double minAbsError = Double.MAX_VALUE;
        double distanceKey = 0;
        for(Map.Entry<Double,Double[]> speedEntry : shootSpeedTable_continuousShooter.entrySet()) {
            tempError = Math.abs(speedEntry.getKey() - distance);

            if (tempError < minAbsError) {
                minAbsError = tempError;
                distanceKey = speedEntry.getKey();
            }
        }

        return distanceKey;
    }

    public double getDistance() {
        return Constants.heightDifference / Math.tan(Math.toRadians(Constants.LimelightMountingAngle + ty.getDouble(0)));
     
    }
    
      
    public double[] getShootingPower() {
        return new double[] {0,0,0};
    }

    public double convertToInches(double meters){
        return meters*39.3701; 
    }

    public double convertRPMToClicks100ms(double velocityRPM){
        return Constants.CLICKS*velocityRPM/60.0/10.0;
    }

    public double convertClicksToRPM(double clicksP100ms) {
        return clicksP100ms/Constants.CLICKS*60.0*10.0;
    }

    public double convertInchesToClicks(double hood_inches){
        return (hood_inches/39.37)*(1.0/Constants.HOOD_CIRCUMFRENCE)*Constants.HOOD_GEAR_RATIO*Constants.CLICKS;
    }

}