package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Hood extends Command {
    private static double startTime;
    private static double timerShot;
    private static boolean aligned = false;
    private boolean articulaAux = false;
    private static boolean indexando = false;
    private static boolean alingOk = false;
    private boolean ctrAligned = true;
            
    /* Shooter INIT */
    public static final TalonFX mShooterR = new TalonFX(22);
    public static final TalonFX mShooterL = new TalonFX(21);
    public static final VelocityVoltage shooterControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMShooter = 0.0;
    /* Shooter END */
    
    /* Hood INIT */
    public static TalonFXS mHood = new TalonFXS(23);
    public static PositionDutyCycle pidCtrHood = new PositionDutyCycle(0);
    /* Hood END */
    
    /* Index INIT */
    public static TalonFX mFeed = new TalonFX(19);
    public static TalonFX mIndex = new TalonFX(20);
    public static final VelocityVoltage indexControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMIndex = 0.0;
    private double targetRPMFeeder = 0.0;
    /* Index END */
    
    /* Belt INIT */
    public static TalonFX mBelt = new TalonFX(18);
    public static final VelocityVoltage beltControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMBelt = 0.0;
    /* Belt END */
    
    public static double OmegaCmd = 0;
    public static double angleTurretSim = 0;
    public static double poseHood = 0, poseHoodSim = 0;
    public double tHigh = 5, tLow = 2, tArticula = 10;
    private boolean RPMShooterOK = false;
    private final class Interpolation{
        private final static double[] distances = {1.2, 1.6, 2.0, 2.4, 2.8, 3.2, 3.6, 4, 4.4, 4.8, 5.2, 5.6};
        private final static double[] RPM = {3700, 3850, 3950, 4100, 4150, 4200, 4250, 4300, 4375, 4500, 4700, 4950};
    }

    PIDController headingPID = new PIDController(0.025, 0.0, 0.001);
    private boolean ctrTimer = true;
    private static Pose2d robotPose;

    public Hood() {
        configHood(0.75, -0.1, 0.45);
        configIndex(NeutralModeValue.Coast);
        configShooter(0.435, NeutralModeValue.Coast);
        configBelt(0.1, -0.1, 0.1);
        headingPID.enableContinuousInput(-180, 180);
        headingPID.setTolerance(2);
    }

    public void execute() {
        double blueX = 4.298;   // Aliança
        double redX = 12.41;    // Aliança

        double targetX = 0;
        double targetY = 0;
        double targetZ = 0;

        double RPMShooter = 0, RPMBelt = 0, RPMIndex = 0;

        double[] velocityShooter = getShooterVelocity();
        double[] velocityIndex = getIndexVelocity();

        /*  Ajuste de target em função da arena - INIT */
        if((!isRedAlliance() && robotPose.getX() <= blueX) || (isRedAlliance() && robotPose.getX() >= redX)){
            targetX = isRedAlliance() ? 11.914 : 4.624;
            targetY = 3.915;
            targetZ = 1.63;
        }
        else if((!isRedAlliance() && robotPose.getX() > blueX + 1.4) || (isRedAlliance() && robotPose.getX() < redX - 1.4)){
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;
            targetX = isRedAlliance() ? 14.32 : 2.331;
            targetZ = 0.5;
        }
        else{
            if((robotPose.getY() - 4.044) >= 0) targetY = 6;
            else targetY = 1.829;
            targetX = isRedAlliance() ? 14.32 : 2.331;
            targetZ = 0.5;
        }
        /*  Ajuste de target em função da arena - END */
        
        double distanceHood = hoodAling(targetX, targetY);

        if((robotPose.getX() >= blueX-0.2 && robotPose.getX() <= (blueX-0.2)+1.2) || (robotPose.getX() >= (redX+0.2) - 1.2 && robotPose.getX() <= redX+0.2)){
            poseHoodSim = -70;
            poseHood = 0;
            RPMShooter = 0;
            RPMIndex = 0;
            RPMBelt = 0;
        }
        else{
            if(isRedAlliance() && (robotPose.getX() <= 4.6) || !isRedAlliance() && (robotPose.getX() >= 11.9)){
                poseHood = 1.1;
                RPMShooter = 6000;
            }
            else{
                poseHoodSim = map(parabola(distanceHood, 1.35, targetX, targetY, targetZ), 40.1, 84, -110, -70);
                poseHood = map(parabola(distanceHood, 1.35, targetX, targetY, targetZ), 84, 40.1, 0.0, 1.85);
                RPMShooter = MathUtil.clamp(interpolate(distanceHood, Interpolation.distances, Interpolation.RPM), 3200, 4950);
            }
        }

        setHoodPosition(poseHood);
        setShooterRPM(RPMShooter);

        SubSystemSIM.setShooterVelocity(3.65);

        RPMShooterOK = (Math.abs(targetRPMShooter - velocityShooter[0]) * 60) < 1000 ? true : false;
        boolean RPMIndexOK = ((targetRPMIndex - velocityIndex[1]) * 60) < 1000 ? true : false;

        if(aligned || alingOk){
            if(ctrAligned) {
                timerShot = Timer.getFPGATimestamp();
                ctrAligned=false;
            }
            if (RPMShooterOK && errorTimer(timerShot) > 0.75) {
                if(ctrTimer){
                    startTime = Timer.getFPGATimestamp();
                    ctrTimer = false;
                }
                RPMIndex = RPMShooter * 0.5;
                RPMBelt = 6000;
                indexando = true;
            }
            else {
                RPMIndex = 0;
                RPMBelt = 0;
                indexando = false;
            }
        }
        else{
            RPMIndex = 0;
            indexando = false;
            RPMBelt = 0;
            articulaAux = false;
            startTime = Timer.getFPGATimestamp();
        }

        setIndexer(RPMIndex, RPMBelt);

        Logger.recordOutput("HOOD/getArticular", getarticulaAux());
        Logger.recordOutput("HOOD/TimerArticular", errorTimer(tArticula));

        Logger.recordOutput("HOOD/RPM/ShooterOK", RPMShooterOK);
        Logger.recordOutput("HOOD/RPM/IndexOK", RPMIndexOK);
        Logger.recordOutput("HOOD/RPM/SetIndex", RPMIndex);
        Logger.recordOutput("HOOD/RPM/SetBelt", RPMBelt);

        Logger.recordOutput("HOOD/RPM/targetShooter", targetRPMShooter * 60);
        Logger.recordOutput("HOOD/RPM/targetIndex", targetRPMIndex *60);
        Logger.recordOutput("HOOD/Ain/SIMAngleParabola", parabola(Robot.auxiliar.getDouble(0), Robot.setHmax.getDouble(1), targetX, targetY, targetZ));
        Logger.recordOutput("HOOD/Ain/AngleParabola", parabola(distanceHood, 1.35, targetX, targetY, targetZ));
        
        Logger.recordOutput("HOOD/Ain/DistanceHUB", distanceHood);
        Logger.recordOutput("HOOD/Ain/IntepolationRPM", interpolate(distanceHood, Interpolation.distances, Interpolation.RPM));
        Logger.recordOutput("HOOD/Position", getHoodPositon());
        Logger.recordOutput("HOOD/RPM/GetShotter1", getShooterVelocity()[0] * 60);
        Logger.recordOutput("HOOD/RPM/GetShooter2", getShooterVelocity()[1] * 60);
        Logger.recordOutput("HOOD/RPM/GetShooter1Acceleration", getShooteracceleration()[0]);
        Logger.recordOutput("HOOD/RPM/GetShooter2Acceleration", getShooteracceleration()[1]);
        
        Logger.recordOutput("HOOD/RPM/GetFeeder", getIndexVelocity()[0] * 60);
        Logger.recordOutput("HOOD/RPM/GetIndex", getIndexVelocity()[1] * 60);
        Logger.recordOutput("HOOD/RPM/GetBelt", getBeltVelocity() * 60);
        Logger.recordOutput("HOOD/Ain/Aligned", aligned);
    }

    public boolean isFinished() {
        return errorTimer(startTime) > 4 && indexando ? true : false;
    }
    
    public static boolean isEnd() {
        return errorTimer(startTime) > 4 && indexando ? true : false;
    }
        
    public void end() {
        setHoodPosition(0);
        setShooterRPM(1000);
        stopIndexSpeed();
        stopBelt();
    
        SubSystemSIM.setShooterVelocity(0);
        aligned=false;
        articulaAux=false;
        indexando = false;
        ctrTimer = true;
        ctrAligned = true;
    
        Logger.recordOutput("HOOD/Ain/Aligned", aligned);
    }
    
    public static void setRobotPose(Pose2d setPose){
        robotPose = setPose;
    }
    
    public static void setAlingAuto(boolean aling){
        alingOk = aling;
    }

    /**
    * @return true se o intake puder articular para ajudar no disparo.
    */
    public boolean getarticulaAux(){
        return articulaAux && Intake.getArticulatedPosition() > 9;
    }

    /**
     * @return o erro do tempo atual em relação ao tempo alvo.
     */
    private static double errorTimer(double tTarget){
        return Timer.getFPGATimestamp() - tTarget;
    }

    public static boolean getAligned(){
            return aligned;
    }

    public static boolean getIndexando(){
            return indexando;
    }

    /**
    * @return um valor linear dentro do range estabelecido.
    */
    public static double getAngleHoodSim(){
        return poseHoodSim;
    }

    /**
    * Controla todos os motores necessarios para conduzir o fuel até o Shooter.
    *
    * @param index Speed do index.
    * @param belt Speed belt.
    */
    private void setIndexer (double RPMIndex, double RPMbelt) {
        setIndexRPM(RPMIndex);
        setFeedRPM(RPMIndex * 0.5);
        setBeltRPM(RPMbelt);
    }

    /**
    * @return um valor linear dentro do range estabelecido.
    *
    * @param x Variavel de leitura.
    * @param in_min Valor minimo de entrada da variavel x.
    * @param in_max Valor maximo de entrada da variavel x.
    * @param out_min Valor de saida minimo permitido (PODE TER B.Ozinhos).
    * @param out_max Valor de saida maximo permitido (PODE TER B.Ozinhos).
    */
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return result;
    }

    /**
    * @return o angulo de saida do objeto de jogo em graus.
    *
    * @param distHoodToHUB Distancia do Hood para o alvo.
    * @param targetH Altura alvo do Fuel (Altura do Fuel no centro do HUB);
    * @param initH Altura inicial da Fuel (Momento de saida do shooter do robô);
    * @param maxH Alura maxima do Fuel durante a trajetória.
    */
    public double parabola (double distHoodToHUB, double hChange, double Target_X, double Target_Y, double Target_Z) {
        Pose2d robot_getValues = robotPose;
        Translation2d robot_pose = robot_getValues.getTranslation();
        Translation2d pose_Target = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0.19719, 0);
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));

        double hHUB = 1.83;

        double targetH = Target_Z;
        double initH = 0.518;
        double maxH = hHUB + hChange;

        double delta_T = 0, a_T = 0, b_T = 0, c_T = targetH - initH;
        double tan_angle_T = 0;

        a_T = (Math.pow(distHoodToHUB, 2) / (4 * (maxH - initH)));
        b_T = -distHoodToHUB;
        delta_T = (Math.pow(b_T, 2) - 4 * a_T * c_T);
        tan_angle_T = (distHoodToHUB + Math.sqrt(delta_T)) / (2 * a_T);

        Pose3d[] traj = trajectoryFuel(
            poseHood,
            pose_Target,
            initH,      // altura inicial (saída do shooter)
            maxH,       // altura máxima da curva
            targetH     // altura do alvo
        );

        Logger.recordOutput("Parabola/ShotTrajectory", traj);
        Logger.recordOutput("Parabola/Angle", Math.toDegrees(Math.atan(tan_angle_T)));

        return Math.toDegrees(Math.atan(tan_angle_T));
    }

    /**
    * Gera uma representação visual da parabola com base nos parametros desejados.
    * @return Pose3d
    * @param robotPos Pose do Hood
    * @param targetPos Pose do alvo a ser mirado.
    * @param startHeight Altura de inicio de saida da parabola.
    * @param peakHeight Altura maxima desejada da parabola.
    * @param endHeight Altura do alvo.
    */
    public Pose3d[] trajectoryFuel(Translation2d robotPos, Translation2d targetPos,
        double startHeight, double peakHeight, double endHeight) {

        List<Pose3d> points = new ArrayList<>();
        int steps = 20;

        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;

            double x = robotPos.getX() + t * (targetPos.getX() - robotPos.getX());
            double y = robotPos.getY() + t * (targetPos.getY() - robotPos.getY());

            double linearZ = startHeight + t * (endHeight - startHeight);
            double arc = 4.0 * t * (1.0 - t) * (peakHeight - ((startHeight + endHeight) / 2.0));

            double z = linearZ + arc;

            points.add(new Pose3d(x, y, z, new Rotation3d()));
        }
        return points.toArray(new Pose3d[0]);
    }

    /**
    * @return a distancia do Hood até o HUB, e tbm gera o valor de Omega para o alinhamento di Chassi.
    *
    * @param Target_X Posição X do alvo a ser atingido.
    * @param Target_Y Posição Y do alvo a ser atingido.
    */
    private double hoodAling (double Target_X, double Target_Y){
        Pose2d robot_getValues = robotPose;
        Rotation2d Robot_Yaw = robot_getValues.getRotation();
        Translation2d robot_pose = robot_getValues.getTranslation();

        Translation2d pose_Target = new Translation2d(Target_X , Target_Y);
        Translation2d offsetHood = new Translation2d(0.19719, 0);
        Translation2d poseHood = robot_pose.plus(offsetHood.rotateBy(Robot_Yaw));
        double distanceHood = poseHood.getDistance(pose_Target);

        Translation2d angleHoodHub = pose_Target.minus(poseHood);
        Rotation2d targetAngleHood = angleHoodHub.getAngle();

        OmegaCmd = headingPID.calculate(Robot_Yaw.getDegrees(), targetAngleHood.getDegrees());

        OmegaCmd = MathUtil.clamp(OmegaCmd, -0.7, 0.7);
        aligned = Math.abs(headingPID.getError()) < 5;

        Logger.recordOutput("ROBOT/Hood", new double[] {poseHood.getX(), poseHood.getY(), Robot_Yaw.getRadians()});
        Logger.recordOutput("ROBOT/OdometryRobot", new double[] {robot_pose.getX(), robot_pose.getY(), Robot_Yaw.getRadians()});
        Logger.recordOutput("FIELD/Target", new double[] {Target_X, Target_Y, Math.toRadians(0)});

        return distanceHood;
    }

    /**
    * @return valor do angulo que o robo deve atingir em Yaw para se alinhar com o HUB.
    */
    public static double getOmega(){
        return OmegaCmd;
    }

    /**
    * @return Interpolação com base na tabela de xs e ys.
    *
    * @param x Variavel de leitura.
    * @param xs Valor xs (Distancia do hood para o HUB).
    * @param ys Valor ys (RPM para acertar o HUB).
    */
    public double interpolate(double x, double[] xs, double[] ys) {
        for(int i=0;i<xs.length-1;i++){
            if(x >= xs[i] && x <= xs[i+1]){

                double ratio =
                    (x - xs[i]) /
                    (xs[i+1] - xs[i]);

                return ys[i] + ratio * (ys[i+1] - ys[i]);
            }
        }
        return ys[ys.length-1];
    }

    /**
    * @return Aliança da Drive Station
    */
    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
    * Configura o motor de inclinação do Hood.
    * @motor 2 x Kraken X60 opostos um ao outro.
    * @param kMode Define o freio do motor coast ou brake.
    */
    public static void configShooter(double kP, NeutralModeValue kMode) {
        
        TalonFXConfiguration cfgShooterL = new TalonFXConfiguration();
        TalonFXConfiguration cfgShooterR = new TalonFXConfiguration();

        cfgShooterL.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfgShooterL.MotorOutput.NeutralMode = kMode;
        cfgShooterL.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterL.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterL.Voltage.PeakForwardVoltage = 12;
        cfgShooterL.Voltage.PeakReverseVoltage = -12;
        cfgShooterL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgShooterL.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        cfgShooterL.Slot0.kP = kP;
        cfgShooterL.Slot0.kI = 0.0;
        cfgShooterL.Slot0.kD = 0.0;

        cfgShooterR.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        cfgShooterR.MotorOutput.NeutralMode = kMode;
        cfgShooterR.CurrentLimits.SupplyCurrentLimit = 80;
        cfgShooterR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgShooterR.Feedback.SensorToMechanismRatio = 1.0;
        cfgShooterR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgShooterR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgShooterR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgShooterR.Voltage.PeakForwardVoltage = 12;
        cfgShooterR.Voltage.PeakReverseVoltage = -12;
        cfgShooterR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfgShooterR.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        cfgShooterR.Slot0.kP = kP;
        cfgShooterR.Slot0.kI = 0.0;
        cfgShooterR.Slot0.kD = 0.0;

        mShooterL.getConfigurator().apply(cfgShooterL);
        mShooterR.getConfigurator().apply(cfgShooterR);
    }
    
    /**
    * @return Posição dos motores do Shooter.
    *
    * @param null
    */
    static public double[] getShooterPosition() {
        return new double[] {
            mShooterL.getPosition().getValueAsDouble(),
            mShooterR.getPosition().getValueAsDouble()};
    }

    /**
    * @return Velocidade dos motores do Shooter.
    *
    * @param null
    */
    static public double[] getShooterVelocity(){
        return new double[] {
            mShooterL.getVelocity().getValueAsDouble(),
            mShooterR.getVelocity().getValueAsDouble()};
    }

    static public double[] getShooterCurrent(){
        return new double[] {
            mShooterL.getSupplyCurrent().getValueAsDouble(),
            mShooterR.getSupplyCurrent().getValueAsDouble()};
    }

    static public double[] getShooteracceleration(){
        return new double[] {
            mShooterL.getAcceleration().getValueAsDouble(),
            mShooterR.getAcceleration().getValueAsDouble()};
    }



    public void setZeroShooter(){
        mShooterL.setPosition(0);
        mShooterR.setPosition(0);
    }

    /**
    * Para os motores do Shooter e configura o motor em modo coast.
    *
    * @param null
    */
    static public void stopShooterSpeed() {
        mShooterL.stopMotor();
        mShooterR.stopMotor();
    }

    /**
    * Define a velocidade dos motores do Shooter
    *
    * @param speedShooter Define a velocidade dos motores do Shotter Left e Right [Limites = 1 a -1].
    */
    static public void setShooterSpeed(double speedShooter){
        mShooterL.set(speedShooter);
        mShooterR.set(speedShooter);
    }
    
    /**
    * Controle PID por velocidade.
    *
    * @param setpointRPM Define a velocidade dos motores do Shotter Left e Right.
    * @param maxRPMKraken 6000.
    */
    public void setShooterRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, 0, 6000);
        this.targetRPMShooter = setpointRPM / 60.0;

        double kS = 0.2;
        double kV = 0.118;
        double kA = 1.0;

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        double VoltageFeedFoward = feedforward.calculate(this.targetRPMShooter);

        //double VoltageFeedFoward = this.targetRPMShooter * kV;

        mShooterL.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
        mShooterR.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
    
        // setpointRPM = MathUtil.clamp(setpointRPM, 0, 6000);
        // this.targetRPMShooter = setpointRPM / 60.0;

        // if(!RPMShooterOK && ctrSlewRate){
        //   targetRPMShooter =  shooterLimiter.calculate(targetRPMShooter);
        //   ctrSlewRate = false;
        // }
        
        // double kV = 0.118;
        // double VoltageFeedFoward = targetRPMShooter * kV;

        // mShooterL.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
        // mShooterR.setControl(shooterControl.withVelocity(targetRPMShooter).withFeedForward(VoltageFeedFoward));
    
    }

    /**
    * Configura o motor de inclinação do Hood.
    * @motor Kraken X44.
    * @param KP Define o ganho proporcial do motor.
    * @param OutMin Saida minima aplicada no motor [Limite = -1].
    * @param OutMax Saida maxima aplicada no motor [Limite = 1].
    */
    public static void configHood(double KP, double OutMin, double OutMax) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfg.MotorOutput.PeakReverseDutyCycle = OutMin;

        cfg.Slot0.kP = KP;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

        mHood.getConfigurator().apply(cfg);
    }
    
    /**
     * @return Posição do Hood
    * Configura o motor de inclinação do Hood.
    * @motor Kraken X44.
    * @param KP Define o ganho proporcial do motor.
    * @param OutMin Saida minima aplicada no motor [Limite = -1].
    * @param OutMax Saida maxima aplicada no motor [Limite = 1].
    */
    static public double getHoodPositon() {
        return mHood.getPosition().getValueAsDouble();
    }

    public void setZeroHood(){
            mHood.setPosition(0);
    }

    /**
    * Define a posição do motor do Hood, considerando tambem não estar de baixo da Treanch
    * @param position Posição do Hood [MIN = 0 - MAX = 1.85].
    */
    public void setHoodPosition(double position) {
        position = MathUtil.clamp(position, 0, 1.85);
        mHood.setControl(pidCtrHood.withPosition(position));
    }

    static public double getHoodCurrent(){
        return mHood.getSupplyCurrent().getValueAsDouble();
    }
    /**
    * Configura o motor de inclinação do Hood.
    * @motor 2 x Kraken X60 opostos um ao outro.
    * @param kMode Define o freio do motor coast ou brake.
    */
    public static void configIndex(NeutralModeValue kMode) {
        
        TalonFXConfiguration cfgFeed = new TalonFXConfiguration();
        TalonFXConfiguration cfgIndex = new TalonFXConfiguration();
        
        cfgFeed.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgFeed.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgFeed.MotorOutput.NeutralMode = kMode;
        cfgFeed.CurrentLimits.SupplyCurrentLimit = 40;
        cfgFeed.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgFeed.Feedback.SensorToMechanismRatio = 1.0;
        cfgFeed.MotorOutput.PeakForwardDutyCycle = 1;
        cfgFeed.MotorOutput.PeakReverseDutyCycle = -1;
        cfgFeed.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgFeed.Voltage.PeakForwardVoltage = 12;
        cfgFeed.Voltage.PeakReverseVoltage = -12;
        cfgFeed.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;

        cfgFeed.Slot0.kP = 0.11;
        cfgFeed.Slot0.kI = 0.0;
        cfgFeed.Slot0.kD = 0.0;

        cfgIndex.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgIndex.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIndex.MotorOutput.NeutralMode = kMode;
        cfgIndex.CurrentLimits.SupplyCurrentLimit = 40;
        cfgIndex.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIndex.Feedback.SensorToMechanismRatio = 1.0;
        cfgIndex.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIndex.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIndex.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIndex.Voltage.PeakForwardVoltage = 12;
        cfgIndex.Voltage.PeakReverseVoltage = -12;
        cfgIndex.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 1;

        cfgIndex.Slot0.kP = 0.11;
        cfgIndex.Slot0.kI = 0.0;
        cfgIndex.Slot0.kD = 0.0;

        mFeed.getConfigurator().apply(cfgFeed);
        mIndex.getConfigurator().apply(cfgIndex);
    }
    
    /**
    * @return Posição dos motores do Indexer.
    *
    * @param null
    */
    static public double[] getIndexPosition() {
        return new double[] {
            mFeed.getPosition().getValueAsDouble(),
            mIndex.getPosition().getValueAsDouble()};
    }

    /**
    * @return Velocidade dos motores do Indexer.
    *
    * @param null
    */
    static public double[] getIndexVelocity() {
        return new double[] {
            mFeed.getVelocity().getValueAsDouble(),
            mIndex.getVelocity().getValueAsDouble()};
    }

    static public double[] getIndexCurrent() {
        return new double[] {
            mFeed.getSupplyCurrent().getValueAsDouble(),
            mIndex.getSupplyCurrent().getValueAsDouble()};
    }

    /**
    * Define a velocidade dos motores do Indexer
    *
    * @param speed Define a velocidade Left e Right [Limites = 1 a -1].
    */
    static public void setIndex(double speed){
        speed = MathUtil.clamp(speed, -1, 1);
        mFeed.set(speed);
        mIndex.set(speed);
    }

    /**
    * Controla os motores do indexer por kV.
    *
    * @param setpointRPM Define o RPM do Indexer.
    */
    public void setIndexRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, -6000, 6000);

        this.targetRPMIndex = setpointRPM / 60.0;
        double kV = 0.15;
        double VoltageFeedFoward = this.targetRPMIndex * kV;
        mIndex.setControl(shooterControl.withVelocity(targetRPMIndex).withFeedForward(VoltageFeedFoward));
    }
    
    /**
    * Controla os motores do feeder por kV.
    *
    * @param setpointRPM Define o RPM do Feeder.
    */
    public void setFeedRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, 0, 6000);

        this.targetRPMFeeder = setpointRPM / 60.0;
        double kV = 0.15;
        double VoltageFeedFoward = this.targetRPMFeeder * kV;
        mFeed.setControl(shooterControl.withVelocity(targetRPMFeeder).withFeedForward(VoltageFeedFoward));
    }

    /**
    * Para os motores do Indexer e configura o motor em modo coast.
    *
    * @param null
    */
    static public void stopIndexSpeed() {
        mFeed.stopMotor();
        mIndex.stopMotor();
    }

    
    /**
    * Configura a posição da articulação do intake.
    * @motor @param type 1 x NEO
    * @param kP Ganho proporcional do sistema.
    * @param OutMin Valor de velocidade minima do motor [LIMITE = -1].
    * @param OutMax Valor de velocidade maxima do motor [LIMITE = 1].
    */
    static void configBelt(double kP, double OutMin, double OutMax){
        TalonFXConfiguration cfgBelt = new TalonFXConfiguration();

        cfgBelt.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgBelt.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgBelt.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgBelt.CurrentLimits.SupplyCurrentLimit = 80;
        cfgBelt.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgBelt.CurrentLimits.SupplyCurrentLimit = 40;
        cfgBelt.Feedback.SensorToMechanismRatio = 1.0;
        cfgBelt.MotorOutput.PeakForwardDutyCycle = 1;
        cfgBelt.MotorOutput.PeakReverseDutyCycle = -1;
        cfgBelt.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgBelt.Voltage.PeakForwardVoltage = 12;
        cfgBelt.Voltage.PeakReverseVoltage = -12;

        mBelt.getConfigurator().apply(cfgBelt);
    }

    /**
    * @return a posição do Belt
    */
    static public double getBeltPosition(){
        return mBelt.getPosition().getValueAsDouble();
    }

    /**
    * @return a velocidade de Belt.
    */
    static public double getBeltVelocity(){
        return mBelt.getVelocity().getValueAsDouble();
    }

    /**
    * @return a velocidade de Belt.
    * @param speed Velocidade do Belt [LIMITE = 1 a -1]
    */
    static public void setBeltSpeed(double speed){
        mBelt.set(speed);
    }

    public void setBeltRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, -6000, 6000);

        this.targetRPMBelt = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPMBelt * kV;
        mBelt.setControl(beltControl.withVelocity(targetRPMBelt).withFeedForward(VoltageFeedFoward));
    }

    /**
    * Desliga o Belt
    */
    static public void stopBelt(){
        mBelt.stopMotor();
    }

    static public double getBeltCurrent(){
        return mBelt.getSupplyCurrent().getValueAsDouble();
    }
}