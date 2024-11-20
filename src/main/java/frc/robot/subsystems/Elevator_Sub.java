// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;
import frc.robot.common.LoadableConfig;

public class Elevator_Sub extends SubsystemBase {
  /** Creates a new Elevator_Sub. */

  DefaultTalonFX.Config elevatorConfig = new DefaultTalonFX.Config("elevatorMotorConfig");
  DefaultTalonFX.Config elevatorConfig2 = new DefaultTalonFX.Config("elevatorMotor2Config");//still "panzerschokolade" at heart

  double kRotationsPerUnitDistance = 9/(1.432)*Math.PI;
  //kNeutralMode = NeutralModeValue.Brake;

  public class Config extends LoadableConfig {
    
    public Config(String filename) {
      double kHomePosition;
      double kMaxUnitsLimit;
      double kMinUnitsLimit;
      double kKp;
      double kKi;
      double kKd;
      double kKa;
      double kKg;
      double kDeadband;
      double kCruiseVelocity;
      double kAcceleration;
      double kRampRate;
      double kMaxForwardOutput;
      double kMaxReverseOutput;
      double kEnableSupplyCurrentLimit;
      boolean kSupplyCurrentLimit;
      double kSupplyCurrentThreshold;
      double kSupplyCurrentTimeout;
      double kHomingZone;
      double kHomingTimeout;
      double kHomingVelocity;
      double kHomingOutput;

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }
  public DefaultTalonFX m_elevatorMotor1 = new DefaultTalonFX(elevatorConfig);
  public DefaultTalonFX m_elevatorMotor2 = new DefaultTalonFX(elevatorConfig2);
  
  public Elevator_Sub() {
    //m_elevatorMotor1.Enable_Sim();
    //m_elevatorMotor2.Enable_Sim();
    //m_elevatorMotor1.Log_Data_To_Smart_Dashboard();
    //m_elevatorMotor2.Log_Data_To_Smart_Dashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_elevatorMotor1.Update();
    //m_elevatorMotor2.Update();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    //m_elevatorMotor1.Update_Sim();
    //m_elevatorMotor2.Update_Sim();
    //m_elevatorMotor1.Duty_Cycle(0.5, false);
    //m_elevatorMotor2.Duty_Cycle(0.5, false);
  }
}
