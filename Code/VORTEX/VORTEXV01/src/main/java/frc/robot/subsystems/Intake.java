package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Intake  extends Command{
    private int varprocess2=0;

    static XboxController driverXbox = new XboxController(0);

    public Intake(){
    }

    @Override
    public void initialize(){
        Constants.Robot_Intake.configFunil.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Intake.configFunil.openLoopRampRate(2);
        Constants.Robot_Intake.configFunil.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Intake.configFunil.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(1, 0.0, 0.0)
        .outputRange(-0.2, 0.2);
    
        Constants.Robot_Intake.mFunil.configure(Constants.Robot_Intake.configFunil, null, null);
        Constants.Robot_Intake.pidControllerFunil = Constants.Robot_Intake.mFunil.getClosedLoopController();
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    public class ColetarCoral extends Thread {      // REVISADO 16/03/2025
        @Override
        public void run(){
            while(!interrupted()){
                if(driverXbox.getXButton()){    // Parar processo
                    varprocess2=0;              // variavel responsavel por cada passo
                    interrupt();
                }
                if(Elevator.getEncoderElevator()<1 && !getSensorIntake() && !Elevator.getSensorOutTake()){  // VERIFICA SE NÃO TEM CONFLITO DE CORAL
                    if(varprocess2==0){     // DESCE O INTAKE
                        if(getPositionInclina()>10){
                            configInclina(0.05, -0.8, 0.8);    // 0.45
                            setPositionInclina(74);
                            varprocess2=1;
                        }
                        else{
                            configInclina(0.01, -0.4, 0.45);    // 0.45
                            setPositionInclina(74);
                        }
                    }
                    if(varprocess2==1){     // LIGA O INTAKE E O FUNIL
                        if(getPositionInclina()<50){
                            configColeta(0.5, 1);
                            setPositionColeta(20000);
                            configFunil(0.5, 0.4);
                            setPositionFunil(2000);
                        } 
                        if(getPositionInclina()>=50){
                            configInclina(0.01, -0.6, 0.6);    // 0.4
                            setPositionInclina(74);
                            varprocess2=2;
                        }
                    }
                    if(varprocess2==2){     // RECOLHE O INTAKE E DESLIGA O FUNIL E INTAKE
                        if(getSensorIntake()){
                            if(getPositionInclina()>50){
                                configInclina(0.05, -0.5, 0.5);
                                setPositionInclina(-2);
                            }
                            if(getPositionInclina()>10 && getPositionInclina()<=50){
                                configInclina(0.05, -0.5, 0.5);
                                setPositionInclina(-2);
                            }
                            if(getPositionInclina()<=10 || driverXbox.getXButton()){
                                configInclina(0.05, -0.3, 0.3);
                                setPositionInclina(-2);
                                stopColeta();
                                Constants.Robot_Intake.mFunil.getEncoder().setPosition(0);
                                setPositionFunil(60);
                                driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
                                varprocess2=0;
                                interrupt();
                            }
                        }
                    }
                }
                else{
                    interrupt();
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }
    public void coletarCoral(){
        varprocess2=0;
        ColetarCoral go = new ColetarCoral();
        go.start();
    }
    public static void setL1(double kp, double mOut, double position){
        configInclina(kp, -mOut, mOut);
        setPositionInclina(position);
    }
    

    public void recolheIntake(){
        configInclina(0.1, -0.45, 0.45);
        setPositionInclina(0);
        stopColeta();
    }
    public static void intakeClimber(){
        configInclina(0.3, -0.45, 0.45);
        setPositionInclina(24);  /// 24 
    }

    public static void autoFunil(){
        configFunil(0.5, 0.5);
        setPosiFunil(2000);
    }

    public static void coletarAlgastep1(){
        configInclina(0.3, -0.3, 0.3);
        setPositionInclina(40);
        configColeta(1, 0.40);
        setPositionColeta(-5000);
    }
    public static void coletarAlgastep2(){
        configInclina(0.3, -0.3, 0.3);
        setPositionInclina(10);  ///30
        configColeta(1, 0.2);  //0.1
        setPositionColeta(-5000);
    }
    public static void depositarAlga(){
        configColeta(0.3, 1);
        setPositionColeta(1500);
    }
    public static void stopFunil(){
        Constants.Robot_Intake.configFunil.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Intake.configFunil.openLoopRampRate(2);
        Constants.Robot_Intake.configFunil.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Intake.configFunil.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(0.2, 0.0, 0.0)
        .outputRange(-0.4, 0.4);

        Constants.Robot_Intake.mFunil.configure(Constants.Robot_Intake.configFunil, null, null);
        Constants.Robot_Intake.pidControllerFunil = Constants.Robot_Intake.mFunil.getClosedLoopController();
        Constants.Robot_Intake.mFunil.getEncoder().setPosition(0);
        Constants.Robot_Intake.pidControllerFunil.setReference(0, ControlType.kPosition);
    }
    static void setPositionFunil(double position){
        Constants.Robot_Intake.pidControllerFunil.setReference(position, ControlType.kPosition);
    }
    public static void setPosiFunil(double position){
        stopFunil();
        configFunil(0.3, 0.3);
        Constants.Robot_Intake.pidControllerFunil.setReference(position, ControlType.kPosition);
    }
    static void setPositionInclina(double position){
        Constants.Robot_Intake.pidControllerInclina.setReference(position, ControlType.kPosition);
    }
    static public void setPosiColeta(double position){
        Constants.Robot_Intake.mColeta.getEncoder().setPosition(0);
        configColeta(1, 0.2);
        Constants.Robot_Intake.pidControllerColeta.setReference(position, ControlType.kPosition);
    }

    public static void setPositionColeta(double position){
        Constants.Robot_Intake.pidControllerColeta.setReference(position, ControlType.kPosition);
    }
    static void stopColeta(){
        configColeta(0.2, 0.2);
        Constants.Robot_Intake.mColeta.getEncoder().setPosition(0);
        setPositionColeta(0);
    }
    static double getPositionFunil(){
        return Constants.Robot_Intake.mFunil.getEncoder().getPosition();
    }
    public static double getPositionInclina(){
        return Constants.Robot_Intake.mInclina.getEncoder().getPosition();
    }
    static double getPositionColeta(){
        return Constants.Robot_Intake.mColeta.getEncoder().getPosition();
    }
    static double getVelocityColeta(){
        return Constants.Robot_Intake.mColeta.getEncoder().getVelocity();
    }
    public static boolean getSensorIntake(){
        return Constants.Robot_Chassy.sensIntake.get();
    }
    public static void setMotorFunil(){
        configFunil(0.3, 0.4);
        setPositionFunil(2000);
    }
    static void configFunil(double P, double Out){
        Constants.Robot_Intake.configFunil.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Intake.configFunil.openLoopRampRate(2);
        Constants.Robot_Intake.configFunil.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Intake.configFunil.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-Out, Out);

        Constants.Robot_Intake.mFunil.configure(Constants.Robot_Intake.configFunil, null, null);
        Constants.Robot_Intake.pidControllerFunil = Constants.Robot_Intake.mFunil.getClosedLoopController();
    }
    static void configInclina(double P, double OutMin, double OutMax){
        Constants.Robot_Intake.configInclina.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Intake.configInclina.openLoopRampRate(50);
        Constants.Robot_Intake.configInclina.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Intake.configInclina.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(OutMin, OutMax);

        Constants.Robot_Intake.mInclina.configure(Constants.Robot_Intake.configInclina, null, null);
        Constants.Robot_Intake.pidControllerInclina = Constants.Robot_Intake.mInclina.getClosedLoopController();
    }
    static void configColeta(double P, double Out){
        Constants.Robot_Intake.configColeta.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Intake.configColeta.openLoopRampRate(2);
        Constants.Robot_Intake.configColeta.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Intake.configColeta.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-Out, Out);

        Constants.Robot_Intake.mColeta.configure(Constants.Robot_Intake.configColeta, null, null);
        Constants.Robot_Intake.pidControllerColeta = Constants.Robot_Intake.mColeta.getClosedLoopController();
    }
}
