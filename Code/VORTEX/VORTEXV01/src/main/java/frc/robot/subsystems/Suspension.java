package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Suspension extends Command{
    
    public Suspension(){
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    static void configSuspension(double P, double Out){
        Constants.Robot_Suspension.configSuspension.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Suspension.configSuspension.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Suspension.configSuspension.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-Out, Out);

        Constants.Robot_Suspension.mPrincipal.configure(Constants.Robot_Suspension.configSuspension, null, null);
        Constants.Robot_Suspension.pidController = Constants.Robot_Suspension.mPrincipal.getClosedLoopController();
    }
    public static void suspensionStep1(){
        configSuspension(1, 1);
        setPositionSuspension(-102);
    }
    public static void suspensionStep2(){
        configSuspension(1, 1);
        setPositionSuspension(300);
    }
    public static void suspensionReturn(){
        configSuspension(1, 1);
        setPositionSuspension(-300);
    }
    public static void setPositionSuspension(double position){
        Constants.Robot_Suspension.pidController.setReference(position, ControlType.kPosition);
    }
}
