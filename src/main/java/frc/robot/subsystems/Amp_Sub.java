// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Amp_Sub extends SubsystemBase {
  /** Creates a new Amp_Sub. */

  DefaultTalonFX.Config ampConfig = new DefaultTalonFX.Config("ampMotor");

  public class Config extends LoadableConfig {

    public boolean StatorCurrentLimitEnable;
    public double StatorCurrentLimit;
    public double PeakForwardVoltage;
    public double PeakReverseVoltage;

    public Config(String filename) {
    
      super.load(this, filename);
      LoadableConfig.print(this);
    }

  }

  public DefaultTalonFX m_ampMotor = new DefaultTalonFX(ampConfig);

  public Amp_Sub() {
    // m_ampMotor.Enable_Sim();
    // m_ampMotor.Log_Data_To_Smart_Dashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_ampMotor.Update();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    // m_ampMotor.Update_Sim();
    // m_ampMotor.Duty_Cycle(0.5, false);
  }
}
