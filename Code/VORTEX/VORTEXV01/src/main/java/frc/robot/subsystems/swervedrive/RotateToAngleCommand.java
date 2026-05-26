package frc.robot.subsystems.swervedrive;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAngleCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final double targetAngleDegrees;
    private static final double TOLERANCE_DEGREES = 2.0; // Tolerância para considerar a rotação concluída
    private static final double kP = 0.1; // Constante de ganho proporcional
    private double error;

    public RotateToAngleCommand(SwerveSubsystem swerveSubsystem, double targetAngleDegrees) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetAngleDegrees = targetAngleDegrees;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Inicializa o erro como a diferença inicial entre o alvo e o ângulo atual
        double currentAngle = swerveSubsystem.getHeading().getDegrees();
        error = targetAngleDegrees - currentAngle;

        // Normaliza o erro para o intervalo [-180, 180]
        error = MathUtil.inputModulus(error, -180, 180);
    }

    @Override
    public void execute() {
        // Calcula o erro atual
        double currentAngle = swerveSubsystem.getHeading().getDegrees();
        error = targetAngleDegrees - currentAngle;

        // Normaliza o erro para o intervalo [-180, 180]
        error = MathUtil.inputModulus(error, -180, 180);

        // Aplica o controle proporcional para calcular a velocidade angular
        double rotationSpeed = kP * error;

        // Garante que a velocidade angular esteja no intervalo
        rotationSpeed = MathUtil.clamp(rotationSpeed, -(Math.PI*1.5), (Math.PI*1.5));
        // Comanda o robô para girar em torno de seu próprio eixo
        swerveSubsystem.drive(new Translation2d(0, 0), rotationSpeed, true);
        
    }

    @Override
    public boolean isFinished() {
        // Termina quando o erro for menor que a tolerância
        return Math.abs(error) < TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Para o movimento assim que o comando termina
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}