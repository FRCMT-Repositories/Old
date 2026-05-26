package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Limelight {

    private static NetworkTable limeTable1;
    private static NetworkTable limeTable2;

    private static final String L1pipe0  = "0";
    private static final String L1pipe1  = "1";
    private final SendableChooser<String> L1m_Pipe = new SendableChooser<>();

    private static final String lime1LED0  = "0";
    private static final String lime1LED1  = "1";
    private static final String lime1LED2  = "2";
    private static final String lime1LED3  = "3";
    private final SendableChooser<String> L1m_LimeLED = new SendableChooser<>();

    public Limelight() {

      L1m_LimeLED.setDefaultOption("LIME1-Pipeline", lime1LED0); 
      L1m_LimeLED.addOption("LIME1-Off", lime1LED1);
      L1m_LimeLED.addOption("LIME1-Piscar", lime1LED2);
      L1m_LimeLED.addOption("LIME1-Force On", lime1LED3);

      L1m_Pipe.setDefaultOption("LIME1-Blue", L1pipe0);
      L1m_Pipe.addOption("LIME1-Red", L1pipe1);

      SmartDashboard.putData("LIME1-PIPELINE SELECT:", L1m_Pipe);
      SmartDashboard.putData(CommandScheduler.getInstance());

      SmartDashboard.putData("LIME1-LED MODE:", L1m_LimeLED);
      SmartDashboard.putData(CommandScheduler.getInstance());

      limeTable1 = NetworkTableInstance.getDefault().getTable("limelight-left");
      limeTable2 = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    public void periodic() {
    }
    public double getTagId(int idLimelight)
    {
        return idLimelight==0 ? limeTable1.getEntry("tid").getDouble(-1.0) : limeTable2.getEntry("tid").getDouble(-1.0);
    }
    public double getTx(int idLimelight)
    {
        return idLimelight==0 ? limeTable1.getEntry("tx").getDouble(0.0) : limeTable2.getEntry("tx").getDouble(0.0);
    }
    public double getTy(int idLimelight)
    {
        return idLimelight==0 ? limeTable1.getEntry("ty").getDouble(0.0) : limeTable2.getEntry("ty").getDouble(0.0);
    }
    public double getTa(int idLimelight)
    {
        return idLimelight==0 ? limeTable1.getEntry("ta").getDouble(0.0) : limeTable2.getEntry("ta").getDouble(0.0);
    }
    public void setPipeline(int idLimelight, int pipeline){
      if(idLimelight==0) limeTable1.getEntry("pipeline").setNumber(pipeline);
      if(idLimelight==1) limeTable2.getEntry("pipeline").setNumber(pipeline);
    }
    public void setPipelineBoth(int pipeline){
      limeTable1.getEntry("pipeline").setNumber(pipeline);
      limeTable2.getEntry("pipeline").setNumber(pipeline);
    }
    public void setLed(int idLimelight, int ledmode){
      if(idLimelight==0) limeTable1.getEntry("ledMode").setNumber(ledmode);
      if(idLimelight==1) limeTable2.getEntry("ledMode").setNumber(ledmode);
    }
}