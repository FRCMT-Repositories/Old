package frc.robot.subsystems.swervedrive;
import edu.wpi.first.math.geometry.Translation2d;

public class Alinhamento {
    private SwerveSubsystem swerve;

    public Alinhamento(SwerveSubsystem swerve){
        this.swerve=swerve;
    }
    public void initialize(){

    }
    public void execute(){
        swerve.drive(new Translation2d(0.0,0.0),0.0,false);
    }

    void isFinished(){
        
    }
    
}
