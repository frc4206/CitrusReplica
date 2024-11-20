// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Pivot_Sub extends SubsystemBase {
  /** Creates a new Pivot_Sub. */
  DefaultTalonFX.Config motorConfig = new DefaultTalonFX.Config("pivotmotor.toml");
  Pivot_Sub.Config cfg;
  
  public static class Config extends LoadableConfig {
    public double kHomePosition;
    public double kExtendedPosition;
    public double kMaxUnitsLimits;
    public double kMinUnitsLimits;
    public double kCruiseVelocity;
    public double kAcceleration;

    public Config(String filename) {
      

			super.load(this, filename);
			LoadableConfig.print(this);
		}
  }

  public DefaultTalonFX m_pivotMotor = new DefaultTalonFX(motorConfig);
  public Pivot_Sub(Pivot_Sub.Config cfg) {
    this.cfg = cfg;
  }

  public void DutyCycle(double p) {
    m_pivotMotor.Duty_Cycle_Output(p);
  }

  public void PID (double pos) {
    m_pivotMotor.Motion_Magic_Pos(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
