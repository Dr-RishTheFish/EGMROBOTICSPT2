package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Relay;
public class VisionCommand extends CommandBase{
    private VisionSubsystem m_visionSubsystem;
    private int pipelineNum;
    public VisionCommand(VisionSubsystem visionSubsystem, int pipelineNum)
    {
        m_visionSubsystem = visionSubsystem;
        this.pipelineNum = pipelineNum;
    }
    @Override
    public void execute() {
        setPipeline();
    }
    public double getActivePipeline() {
        return VisionSubsystem.visionSwitch.getEntry("getpipe").getDouble(0);
    }
    public void setPipeline(){
        VisionSubsystem.visionSwitch.getEntry("pipeline").setNumber(pipelineNum);
    }
}