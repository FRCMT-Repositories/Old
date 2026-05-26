package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class JPmath {
    
  public static double PID(double Position, double Position_Desejada, double P, double OutputRange, double Position_Range)
  {
    double Value;
    Value = ((Position_Desejada)-(Position))*P;
      
    if((Value < Position_Range) && (Value > -Position_Range)) 
    {
      Value = 0;
    }
    if(Value > OutputRange) Value = OutputRange;
    if(Value < OutputRange*(-1)) Value = OutputRange*(-1);
    return Value;
  }

  public static double PID_Wrapping(double Position, double Position_Desejada, double P, double OutputRange, double Position_Range, double Range_Warapping)
  {
    double Value;
    if(((Position_Desejada - Position)>(Range_Warapping/2)) || (Position_Desejada - Position)<(-Range_Warapping/2))
    {
      Value = (Position - Position_Desejada) * P;
    }
    else
    {
      Value = (Position_Desejada-Position)*P;
    }
    if((Position_Desejada-Position) < Position_Range && (Position_Desejada-Position) > -Position_Range) 
    {
      Value = 0;
    }
    if(Value > OutputRange) Value = OutputRange;
    if(Value < OutputRange*(-1)) Value = OutputRange*(-1);
    return Value;
  }

  public static double getJoy_degrees(XboxController controller, int Axisx, int Axisy) { 
    return Math.toDegrees(
      Math.atan2(controller.getRawAxis(Axisx), 
      controller.getRawAxis(Axisy)));
  }
  
  public static boolean getJoyStimulated(XboxController controller, int Axisx, int Axisy, double Range) {
    return (controller.getRawAxis(Axisx) <= (-Range) 
    || controller.getRawAxis(Axisx) >= Range 
    || controller.getRawAxis(Axisy) <= (-Range) 
    || controller.getRawAxis(Axisy) >= Range);
  }
  
  public static boolean getJoyStimulated(XboxController controller, int Axis, double Range) {
    return (controller.getRawAxis(Axis) <= (-Range) 
    || controller.getRawAxis(Axis) >= Range);
  }
  
}
