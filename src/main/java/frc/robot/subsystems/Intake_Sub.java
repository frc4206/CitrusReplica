// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Intake_Sub extends SubsystemBase {
  /** Creates a new Intake_Sub. */
public double kRotationPerUnitDistance = (1/360)*45;

  DefaultTalonFX.Config motorConfig = new DefaultTalonFX.Config("intakemotor.toml");
  
  public static class Config extends LoadableConfig {
    public double intakeVoltage;
    public double outakeVoltage;
    public double SupplyCurrentLimit;
    public double PeakForwardVoltage;
    public double PeakReverseVoltage;
    public boolean SupplyCurrentLimitEnable;

    public Config(String filename) {

			super.load(this, filename);
			LoadableConfig.print(this);
		}
  }
  
  public DefaultTalonFX m_intakeMotor = new DefaultTalonFX(motorConfig);
  
  public Intake_Sub(Intake_Sub.Config cfg) {
    m_intakeMotor.Enable_Sim();
    m_intakeMotor.Log_Data_To_Smart_Dashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intakeMotor.Update();
  }

  public void SetVoltage(double volts) {
    m_intakeMotor.Voltage_Output(volts);
  }


  @Override
  public void simulationPeriodic() {
      super.simulationPeriodic();
      m_intakeMotor.Update_Sim();
      //m_intakeMotor.Duty_Cycle(0.5, false);
  }
}
