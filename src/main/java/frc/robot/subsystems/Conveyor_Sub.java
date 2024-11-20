// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;


public class Conveyor_Sub extends SubsystemBase {
  /** Creates a new Conveyor_Sub. */
  DefaultTalonFX.Config motorConfig = new DefaultTalonFX.Config("conveyorMotor");
  double kFeederVelocity = 60*(25/24);
  public class Config extends LoadableConfig{

    public Config(String filename){ 

      double StatorCurrentLimitEndable;
      double StatorCurrentLimitEnable;
      double PeakForwardVoltage;
      double PeakReverseVoltage;

      super.load(this, filename); 
      LoadableConfig.print(this); 
    }
  }

  public DefaultTalonFX m_conveyorMotor = new DefaultTalonFX(motorConfig);

  public Conveyor_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
