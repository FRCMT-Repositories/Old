package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;


public class Intake {

    /* Intake INIT */
    public static TalonFX mIntakeL = new TalonFX(14);
    public static TalonFX mIntakeR = new TalonFX(15);
    public static final VelocityVoltage intakeControl = new VelocityVoltage(0).withSlot(0);
    private double targetRPMIntake = 0.0;

    public static TalonFX mArticulated = new TalonFX(16);
    public static PositionDutyCycle pidCtrArticulated = new PositionDutyCycle(0);
    /* Intake END */

    public Intake(){
        configArticulated(0.1, -0.1, 0.1, NeutralModeValue.Brake);
        configIntake(0.1, -0.1, 0.1);
    }

    /**
    * Configura o intake.
    * @motor @param type 2 x Kraken X60
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Valor de saida minimo do motor [LIMITE = -1].
    * @param OutMax Valor de saida maximo do motor [LIMITE = 1].
    */
    static void configIntake(double KP, double OutMin, double OutMax){
        TalonFXConfiguration cfgIntakeL = new TalonFXConfiguration();
        TalonFXConfiguration cfgIntakeR = new TalonFXConfiguration();

        cfgIntakeL.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgIntakeL.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIntakeL.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgIntakeL.CurrentLimits.SupplyCurrentLimit = 40;
        cfgIntakeL.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIntakeL.Feedback.SensorToMechanismRatio = 1.0;
        cfgIntakeL.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIntakeL.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIntakeL.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIntakeL.Voltage.PeakForwardVoltage = 12;
        cfgIntakeL.Voltage.PeakReverseVoltage = -12;

        cfgIntakeR.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfgIntakeR.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;

        cfgIntakeR.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfgIntakeR.CurrentLimits.SupplyCurrentLimit = 40;
        cfgIntakeR.CurrentLimits.SupplyCurrentLimitEnable = false;
        cfgIntakeR.Feedback.SensorToMechanismRatio = 1.0;
        cfgIntakeR.MotorOutput.PeakForwardDutyCycle = 1;
        cfgIntakeR.MotorOutput.PeakReverseDutyCycle = -1;
        cfgIntakeR.MotorOutput.DutyCycleNeutralDeadband = 0;
        cfgIntakeR.Voltage.PeakForwardVoltage = 12;
        cfgIntakeR.Voltage.PeakReverseVoltage = -12;

        mIntakeL.getConfigurator().apply(cfgIntakeL);
        mIntakeR.getConfigurator().apply(cfgIntakeR);
    }

    /**
    * @return a posição dos coletores do intake.
    */
    static public double[] getIntakePosition(){
        return new double[] {
            mIntakeL.getPosition().getValueAsDouble(),
            mIntakeR.getPosition().getValueAsDouble()};
    }

    static public double[] getIntakeCurrent(){
        return new double[] {
            mIntakeL.getSupplyCurrent().getValueAsDouble(),
            mIntakeR.getSupplyCurrent().getValueAsDouble()};
    }

    /**
    * @return a velocidade dos coletores do intake.
    */
    static public double[] getIntakeVelocity(){
        return new double[] {
            mIntakeL.getVelocity().getValueAsDouble(),
            mIntakeR.getVelocity().getValueAsDouble()};
    }
    /**
    * Retorna a velocidade dos coletores do intake.
    * @param speed Valor da velocidade do coletor [LIMITE = 1 a -1]
    */
    public void setIntakeSpeed(double speed){
        speed = MathUtil.clamp(speed, -1, 1);
        mIntakeL.set(speed);
        mIntakeR.set(speed);
    }
    
    /**
    * Retorna a velocidade dos coletores do intake.
    * @param speed Valor da velocidade do coletor [LIMITE = 1 a -1]
    */
    static public void intakeStop(){
        mIntakeL.stopMotor();
        mIntakeR.stopMotor();
    }

    /**
    * Controle PID por velocidade.
    *
    * @param setpointRPM Define a velocidade dos motores do Intake Left e Right.
    * @param maxRPMKraken 6000.
    */
    public void setIntakeRPM(double setpointRPM){
        setpointRPM = MathUtil.clamp(setpointRPM, -6000, 6000);

        this.targetRPMIntake = setpointRPM / 60.0;
        double kV = 0.115;
        double VoltageFeedFoward = this.targetRPMIntake * kV;
        mIntakeL.setControl(intakeControl.withVelocity(targetRPMIntake).withFeedForward(VoltageFeedFoward));
        mIntakeR.setControl(intakeControl.withVelocity(targetRPMIntake).withFeedForward(VoltageFeedFoward));
    }

    /**
    * Configura a posição da articulação do intake.
    * @motor @param type 1 x Kraken X60
    * @param KP Ganho proporcional do sistema.
    * @param OutMin Valor de saida minimo do motor [LIMITE = -1].
    * @param OutMax Valor de saida maximo do motor [LIMITE = 1].
    */
    public static void configArticulated(double KP, double OutMin, double OutMax, NeutralModeValue kMode) {
       TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotorOutput.PeakForwardDutyCycle = OutMax;
        cfg.MotorOutput.PeakReverseDutyCycle = OutMin;
        cfg.MotionMagic.MotionMagicAcceleration = 500; // RPS/s
        cfg.MotionMagic.MotionMagicCruiseVelocity = 500; // RPS

        cfg.Slot0.kP = KP;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        cfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
        cfg.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;

        mArticulated.getConfigurator().apply(cfg);

    }

    /**
    * @return a posição da articulação do intake.
    */
    public static double getArticulatedPosition() {
        return mArticulated.getPosition().getValueAsDouble();
    }

    public void setZeroArticulated(){
            mArticulated.setPosition(0);
    }

    /**
    * @return a velocidade da articulação do intake.
    */
    public static double getArticulatedVelocity() {
        return mArticulated.getVelocity().getValueAsDouble();
    }

    /**
    * Configura a posição da articulação do intake.
    * @param kP Ganho proporcional do sistema.
    * @param position Valor da posição desejada [MIN = 0 - MAX = 22.394].
    * @param speed Valor de velocidade maxima e minima do motor [LIMITE = 1 a -1].
    */
    public static void setArticulated(double kP, double position, double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        position = MathUtil.clamp(position, 0, 22.394);

        configArticulated(kP, -speed, speed, NeutralModeValue.Brake);
        mArticulated.setControl(pidCtrArticulated.withPosition(position));
    }

    /**
    * Deixa o motor livre.
    */
    static public void stopArticulated(){
        configArticulated(0, 0, 0, NeutralModeValue.Coast);
        mArticulated.set(0);
    }
}
