package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends Command{
    static XboxController driverXbox = new XboxController(0);
    static Joystick m_Joy = new Joystick(2);

    private double posilevel;

    private double posialga;
    private double posielevator;

    private static int varprocess5=0; // Coral Station

    private static int varprocess1=0; // Position
    private static int varprocess2=0; // Position
    private static int varprocess6=0; // Position
    private static boolean varctr=false;

    private static Boolean corrigeL4Down=false;
    

    private static int varprocess3=0; // Alga

    private static int contagemOuttake=0;
    public Elevator(){
    }
    
    @Override
    public void initialize(){
        driverXbox.setRumble(RumbleType.kBothRumble, 0);
        
        configElevator(0.1, 1);
        configOutInc(0.1, 1);
        configOuttake(0.08, 0.15);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    public static void pararOuttake(){
        Constants.Robot_Elevator.configOuttake.inverted(false).idleMode(IdleMode.kCoast);
        Constants.Robot_Elevator.configOuttake.openLoopRampRate(2);
        Constants.Robot_Elevator.configOuttake.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configOuttake.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0.0, 0.0)
        .outputRange(-0.5, 0.5);

        Constants.Robot_Elevator.mOuttake.configure(Constants.Robot_Elevator.configOuttake, null, null);
        Constants.Robot_Elevator.pidControllerOuttake = Constants.Robot_Elevator.mOuttake.getClosedLoopController();

        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
        Constants.Robot_Elevator.pidControllerOuttake.setReference(0, ControlType.kPosition);
    }
    public static void stopOuttake(){
        configOuttake(0.3, 0.5);
        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
        setPositionOuTake(0);
    }
    public static void depositar(){
        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
        driverXbox.setRumble(RumbleType.kBothRumble, 0);

        if(getEncoderElevator()> 23){
            if(driverXbox.getRawAxis(2)>0.2){
                configOuttake(0.5,1);
                if(getEncoderOutInc()<-0.6) setPositionOuTake(20);
                else setPositionOuTake(-20);
            }
            else{
                configOuttake(0.3,0.5);
                if(getEncoderOutInc()<-0.6) setPositionOuTake(0.9);
                else setPositionOuTake(-0.9);
            }
        }
        if(getEncoderElevator()> 7.5 && getEncoderElevator()<=23){
            configOuttake(0.55,0.23);  // deposito = 1 antes
            setPositionOuTake(-30);
        }
        if(getEncoderElevator()<= 7.5){
            if(driverXbox.getRawAxis(2)>0.2){
                configOuttake(0.5,1);
                setPositionOuTake(-20);
            }
            else{
                configOuttake(0.3,0.5);
                setPositionOuTake(-1);
            }
        }
    }

    public class CoralStation extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                if(Intake.getSensorIntake() || RobotContainer.elevatorManipulation()){
                    interrupt();
                }
                else{
                    if(varprocess5==0){
                        if(getSensorOutTake()){
                            varprocess5=6;
                            driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
                            interrupt();
                        }
                        configOuttake(0.5, 0.35);
                        configElevator(0.25, 1);
                        configOutInc(0.6, 1);
                        setPositionOuTake(2000);
                        varprocess5=1;
                    }
                    if(varprocess5==1){
                        setPositionElevator(13);//12.25
                        if(getEncoderElevator()>5){
                            setPositionOutInc(-1.26);
                            varprocess5=2;
                        }
                    }
                    if(varprocess5==2){
                        if(getSensorOutTake()==true) {
                            driverXbox.setRumble(RumbleType.kBothRumble, 0.5);
                            stopOuttake();//pararOuttake();
                            varprocess5=3;
                        }
                    }
                    if(varprocess5==3){
                        setPositionElevator(14.5);
                        if(getEncoderElevator()>13.5){  ////14
                            pararOuttake();//pararOuttake();
                            configOutInc(1, 1);
                            setPositionOutInc(0.6);
                            varprocess5=4;
                        }
                    }
                    if(varprocess5==4){
                        if(getEncoderOutInc()>-0.3){
                            configOuttake(1,0.5);  //0.1
                            setPositionOuTake(0.5);
                            if(getEncoderOutTake() > 0.3){
                                Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
                                configOuttake(0.2,0.5);
                                varprocess5=5;
                            }
                        }
                    }
                    if(varprocess5==5){
                        setPositionOuTake(-0.4);
                        positionDeposit(0.4);
                        varprocess5=0;
                        interrupt();
                    }
                    SmartDashboard.putNumber("VAR", varprocess5);
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
    public void coralStation(){
        varprocess5=0;
        CoralStation pegar = new CoralStation();
        pegar.start();
    }

    public class PositionDeposit extends Thread {   // REVISADO 16/03/2025
        @Override
        public void run(){
            while(!interrupted()){

                if(Intake.getSensorIntake()){           // SE TIVER CORAL NO FUNIL ELE GOSPE O CORAL PRA FORA
                    Intake.setPosiFunil(-150);
                    if(getEncoderElevator()>=Constants.Field.CORAL_L1){
                        configOutInc(0.1, 0.35); // 0.1
                        setPositionOutInc(0.3);
                        Timer.delay(1);
                        configElevator(0.2, 0.3, 0.8);
                        setPositionElevator(Constants.Field.CORAL_L1);
                    }
                    interrupt();
                }
                
                if(!Intake.getSensorIntake() && driverXbox.getLeftTriggerAxis()<=0.3){
                    if(varprocess1==0){         // Inicialização
                        varctr=false;
                        corrigeL4Down=false;
                        varprocess6=0;
                        configElevator(0.2, 0.4,0.8); ///0.13******************** */
                        configBreakOutInc();
                        varprocess1=1;
                    }
                    if(varprocess1==1){
                        if(posilevel > 25){      // L4
                            if(varprocess2==0){     // SOBE O ELEVADOR
                                stopOuttake();
                                setPositionElevator(posilevel-1.25);
                                if(getEncoderElevator()>(posilevel-1.7)) varprocess2=1;
                            }
                            if(varprocess2==1){     // freio
                                varctr=false;
                                configElevator(0.25, 0.4,0.6);
                                setPositionElevator(posilevel);
                                varprocess2=2;
                            }

                            if(varprocess2==2){     // JOGA O CORAL UM PUCO PRA FORA E DEPOIS DA UMA LEVE INCLINADA
                                if(getEncoderElevator()>20 && varctr==false){
                                    double extrude = -0.4;   ///0.6

                                    configOuttake(0.2, 0.55); //configOuttake(0.2, 0.4);
                                    configOutInc(0.1, 1);
                                    setPositionOuTake(extrude);
                                    if(getEncoderOutTake()>=extrude+0.2){
                                        Timer.delay(0.2);  /// 0.4s deu bom
                                        stopOuttake();
                                        setPositionOutInc(-2.67);
                                        varctr=true;
                                        varprocess2=3;
                                    }
                                }
                            }
                            if(varprocess2==3){     // TEMINA DE INCLINAR O OUTTAKE E DEPOIS GOSPE ELE UM POUCO PRA FORA
                                if(getEncoderElevator()>25.5){
                                    configBreakElevator();
                                }
                                if(getEncoderElevator()>25 && getEncoderOutInc()<-1.8){
                                    double incExtrude=-5.5;         //double incExtrude=-4.8;    /// -5.3335;

                                    configOutInc(0.25, 1);        //configOutInc(0.5, 1);
                                    setPositionOutInc(incExtrude);
                                    if(getEncoderOutInc()<incExtrude+0.2){
                                        double extrude=0.4;
                                        configOuttake(0.6, 0.75);
                                        setPositionOuTake(extrude);
                                        if(getEncoderOutTake()<extrude-0.2){
                                            stopOuttake();
                                            varprocess1=0;
                                            varprocess2=0;
                                            interrupt();
                                        }
                                    }
                                }
                            }
                            SmartDashboard.putNumber("Passo", varprocess2);
                        }
                        else{                     // L1, L2, L3 ou inferior
                            if(varprocess6==0){ // RETORNA O OUTTAKE PARA PARA DEPOSITO NOS NIVEIS DE L1, L2 E L3
                                if(getEncoderOutInc()<-1){
                                    configOutInc(0.1, 0.1);
                                    setPositionOutInc(0.3);
                                }
                                else{
                                    configBreakOutInc();
                                    setPositionOutInc(0.3);
                                    if(getEncoderElevator()>22.5){
                                        corrigeL4Down=true;
                                    }
                                    varprocess6=1;
                                }
                                }
                            if(varprocess6==1){
                                if(getEncoderOutInc() > -4.2){
                                    if((posilevel > 5.25) && (posilevel < 30)){// Qualquer nivel que não seja abaixo do vale
                                        double amortecedor = 2;
                                        configElevator(0.13, 0.4,1);
                                        setPositionElevator(posilevel);
                                        if((posilevel+1.25)>getEncoderElevator()){
                                            if(getEncoderElevator()>(posilevel-amortecedor)){
                                                configBreakElevator();
                                            }
                                            else{
                                                configElevator(0.1, 0.1,1);
                                            }
                                        }
                                        if(posilevel-(1.25)<getEncoderElevator()){
                                            if(getEncoderElevator()<posilevel+amortecedor){
                                                configBreakElevator();
                                            }
                                            else{
                                                configElevator(0.1, 0.1,1);
                                            }
                                        }
                                        double rangeFinish=0.25;
                                        if(getEncoderElevator()>=posilevel-rangeFinish && getEncoderElevator()<=posilevel+rangeFinish){
                                            if(corrigeL4Down){
                                                configOuttake(0.2, 0.4); //configOuttake(0.2, 0.4);
                                                setPositionOuTake(0.6);
                                            }
                                            varprocess1=0;
                                            interrupt();
                                        }
                                    }
                                    else{   // Abaixo do vale
                                        double amortecedor = 4;
                                        stopOuttake();
                                        if(getEncoderElevator()>=amortecedor){
                                            if(!getSensorOutTake()){
                                                configElevator(0.1, 0.7,1);
                                                setPositionElevator(amortecedor);
                                            }
                                            else
                                            {
                                                configOutInc(0.1, 1);
                                                setPositionOutInc(0.6);
                                                if(getEncoderOutInc()>(-0.5)){  /// tava 1
                                                    configElevator(0.1, 0.7,1);
                                                    setPositionElevator(amortecedor);
                                                    varprocess1=0;
                                                    varprocess6=0;
                                                    interrupt();
                                                }
                                            }
                                        }
                                        /*
                                        if(getSensorOutTake() && (getEncoderOutInc() > -1.2)){
                                            configBreakOutInc();
                                            setPositionElevator(posilevel);
                                            if(getEncoderElevator()<amortecedor) 
                                            {
                                                configBreakElevator();
                                            }

                                            if(posilevel>=amortecedor){
                                                varprocess1=0;
                                                interrupt();
                                            }
                                        }*/
                                        // if(!getSensorOutTake() && (getEncoderOutInc() > -0.6)){
                                        //     setPositionElevator(posilevel);
                                        // }
                                        if(!getSensorOutTake() && (getEncoderOutInc() > -0.6 && posilevel<amortecedor && getEncoderElevator()<amortecedor)){
                                            configBreakElevator();
                                            setPositionElevator(posilevel);
                                            if(corrigeL4Down){
                                                configOuttake(0.2, 0.4); //configOuttake(0.2, 0.4);
                                                setPositionOuTake(-0.6);
                                            }
                                            varprocess1=0;
                                            varprocess6=0;
                                            interrupt();
                                        }
                                    }
                                }
                            }
                        }
                    }
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
    public void positionDeposit(double posilevel){
        varprocess1=0;
        varprocess2=0;
        varprocess6=0;
        this.posilevel=posilevel;
        PositionDeposit position = new PositionDeposit();
        position.start();
    }
    public class ColetarAlga extends Thread {       // REVISADO 16/03/2025
        @Override
        public void run(){
            while(!interrupted()){
                if((driverXbox.getLeftTriggerAxis()>0.3 && Intake.getSensorIntake())  || RobotContainer.elevatorManipulation()){
                    interrupt();
                }
                else{
                    if(varprocess3==0){     // INICIALIZAÇÃO
                        contagemOuttake=0;
                        configElevator(0.1,0.2, 0.75);///**************************************** */
                        configOutInc(0.35,0.75); // .outputRange(-1, 0.2);
                        configOuttake(0.2,0.5);  /// 0.5
                        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
                        setPositionOuTake(2000);
    
                        varprocess3=1;
                    }
                    if(varprocess3==1){     // MOVE ELEVADOR
                        setPositionElevator(posielevator);
                        varprocess3=2;
                    }
                    if(varprocess3==2){     // INCLINA OUTTAKE
                        if(getEncoderElevator()>=Constants.Field.CORAL_L1){
                            setPositionOutInc(posialga+1);
                            varprocess3=3;
                        }
                    }
                    if(varprocess3==3){
                        if(getEncoderOutInc()<(posialga+1.2)){
                            configOutInc(0.35,0.4);
                            setPositionOutInc(posialga);
                        }
                    }

                    if(varprocess3==3){     // VERIFICA SE A ALGA FOI COLETADA E FAZ O PROCESSO DE COLETA
                        // if(getOutTakeVelocity()>-10){
                        //     contagemOuttake++;
                        // }
                        if(driverXbox.getRawAxis(2)>0.2 || RobotContainer.autoAlgaeTransport){
                            varprocess3=0;
                            configElevator(0.1,0.05, 0.4);///**************************************** */
                            configOutInc(1,1);
                            setPositionElevator(1.25);
                            configOuttake(0.2,0.2);
                            setPositionOuTake(2000);
                            SmartDashboard.putNumber("Cont2", contagemOuttake);
                            interrupt();
                        }
                    }
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
    public void coletarAlga(double posielevator, double posialga){
        varprocess3=0;
        contagemOuttake=0;
        this.posialga=posialga;
        this.posielevator=posielevator;
        ColetarAlga position = new ColetarAlga();
        position.start();
    }
    public class RemoverAlga extends Thread {       // REVISADO 16/03/2025
        @Override
        public void run(){
            while(!interrupted()){
                if(Intake.getSensorIntake() || RobotContainer.elevatorManipulation()){
                    interrupt();
                }
                else{
                    if(varprocess3==0){             // INICIALIZAÇÃO
                        contagemOuttake=0;
                        configElevator(0.1,0.2, 1);
                        configOutInc(0.2,1); // .outputRange(-1, 0.2);
                        configOuttake(1,1);
                        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
                        setPositionOuTake(-2000);
                        varprocess3=1;
                    }
                    if(varprocess3==1){             // MOVE ELEVADOR
                        setPositionElevator(posielevator);
                        varprocess3=2;
                    }
                    if(varprocess3==2){             // INCLINA OUTTAKE
                        if(getEncoderElevator()>=Constants.Field.CORAL_L1){     ///3.5
                            setPositionOutInc(posialga);
                            varprocess3=0;
                            interrupt();
                        }
                    }
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
    public void removerAlga(double posielevator, double posialga){
        varprocess3=0;

        this.posialga=posialga;
        this.posielevator=posielevator;
        RemoverAlga position = new RemoverAlga();
        position.start();
    }

    static void configOuttake(double P, double Out){
        Constants.Robot_Elevator.configOuttake.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configOuttake.openLoopRampRate(1);
        Constants.Robot_Elevator.configOuttake.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configOuttake.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-Out, Out);

        Constants.Robot_Elevator.mOuttake.configure(Constants.Robot_Elevator.configOuttake, null, null);
        Constants.Robot_Elevator.pidControllerOuttake = Constants.Robot_Elevator.mOuttake.getClosedLoopController();
    }
    static void configOutInc(double P, double Out){
        Constants.Robot_Elevator.configOutInc.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configOutInc.openLoopRampRate(1);
        Constants.Robot_Elevator.configOutInc.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configOutInc.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-Out, Out);

        Constants.Robot_Elevator.mOutInc.configure(Constants.Robot_Elevator.configOutInc, null, null);
        Constants.Robot_Elevator.pidControllerOutInc = Constants.Robot_Elevator.mOutInc.getClosedLoopController();
    }
    static void configElevator(double P, double Out){
        Constants.Robot_Elevator.configElevator.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configElevator.openLoopRampRate(2);
        Constants.Robot_Elevator.configElevator.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configElevator.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-0.75, Out);

        Constants.Robot_Elevator.mPrincipal.configure(Constants.Robot_Elevator.configElevator, null, null);
        Constants.Robot_Elevator.pidControllerElevator = Constants.Robot_Elevator.mPrincipal.getClosedLoopController();
    }
    static void configElevator(double P, double OutMin, double OutMax){
        Constants.Robot_Elevator.configElevator.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configElevator.openLoopRampRate(2);
        Constants.Robot_Elevator.configElevator.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configElevator.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(P, 0.0, 0.0)
        .outputRange(-OutMin, OutMax);

        Constants.Robot_Elevator.mPrincipal.configure(Constants.Robot_Elevator.configElevator, null, null);
        Constants.Robot_Elevator.pidControllerElevator = Constants.Robot_Elevator.mPrincipal.getClosedLoopController();
    }
    public static double getEncoderElevator(){
        return Constants.Robot_Elevator.mPrincipal.getEncoder().getPosition();
    }
    static double getEncoderOutInc(){
        return Constants.Robot_Elevator.mOutInc.getEncoder().getPosition();
    }
    static double getEncoderOutTake(){
        return Constants.Robot_Elevator.mOuttake.getEncoder().getPosition();
    }
    static boolean getSensorOutTake(){
        return Constants.Robot_Elevator.sensOuttake.get();
    }
    static boolean getSensorInTake(){
        return Constants.Robot_Chassy.sensIntake.get();
    }
    static void setPositionElevator(double position){
        Constants.Robot_Elevator.pidControllerElevator.setReference(position, ControlType.kPosition);
    }
    static void setPositionOutInc(double position){
        Constants.Robot_Elevator.pidControllerOutInc.setReference(position, ControlType.kPosition);
    }
    public static void setPositionOuTake(double position){
        Constants.Robot_Elevator.pidControllerOuttake.setReference(position, ControlType.kPosition);
    }
    public static void setPositionOuTake(double kp, double mOut, double position){
        configOuttake(kp, mOut);
        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
        Constants.Robot_Elevator.pidControllerOuttake.setReference(position, ControlType.kPosition);
    }
    public static void setPosiOuTake(double position){
        Constants.Robot_Elevator.mOuttake.getEncoder().setPosition(0);
        configOuttake(0.1, 0.2); //0.7
        Constants.Robot_Elevator.pidControllerOuttake.setReference(position, ControlType.kPosition);
    }
    public static void configBreakElevator(){
        Constants.Robot_Elevator.configElevator.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configElevator.openLoopRampRate(0.5);
        Constants.Robot_Elevator.configElevator.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configElevator.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(0.2, 0.0, 0.0)
        .outputRange(-0.055, 1);
        
        Constants.Robot_Elevator.mPrincipal.configure(Constants.Robot_Elevator.configElevator, null, null);
        Constants.Robot_Elevator.pidControllerElevator = Constants.Robot_Elevator.mPrincipal.getClosedLoopController();
    }
    public static void configBreakOutInc(){
        Constants.Robot_Elevator.configOutInc.inverted(false).idleMode(IdleMode.kBrake);
        Constants.Robot_Elevator.configOutInc.openLoopRampRate(1);
        Constants.Robot_Elevator.configOutInc.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        Constants.Robot_Elevator.configOutInc.closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0.0, 0.0)
        .outputRange(-0.075, 0.075);
        
        Constants.Robot_Elevator.mOutInc.configure(Constants.Robot_Elevator.configOutInc, null, null);
        Constants.Robot_Elevator.pidControllerOutInc = Constants.Robot_Elevator.mOutInc.getClosedLoopController();
    }
    public static double getOutTakeVelocity(){
        return Constants.Robot_Elevator.mOuttake.getEncoder().getVelocity();
    }
}
