// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

 // --- MOTORES ---
  public static final int m_MotorRodaSup = 6;
  public static final int m_MotorColeta = 8;
  public static final int m_MotorRodaMeio = 7;
  
  // --- CLIMB ---
  public static final int m_MotorClimb = 9;

  // --- POSIÇÕES DO HUB (MIRA GLOBAL) ---
  public static final double HUB_X_Red = 12.05; 
  public static final double HUB_Y_Red = 4.03;

  public static final double HUB_X_Blue = 4.6; 
  public static final double HUB_Y_Blue = 4.03;

  // --- Limite de velocidade PID Heading e Mira
  public static final double LIMITE_ROTACAO = 4;

  // ==========================================================
  //      CALIBRAÇÃO DO FATOR DE ESCALA DO PIGEON 2.0
  // ==========================================================
  // Baseado no teste de levantar o robô e dar 6 voltas:
  // O robô deu 6 voltas físicas (2160º), mas o Pigeon leu 2172º.
  public static final double GRAUS_REAIS = 1800.0;
  public static final double GRAUS_PIGEON = 1790.0;
  
  // Este fator será usado automaticamente no Drivetrain e no Piloto
  public static final double FATOR_ESCALA_PIGEON = GRAUS_REAIS / GRAUS_PIGEON;
  
}