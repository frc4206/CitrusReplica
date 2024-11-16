// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;
import frc.robot.common.LoadableConfig;


public class Shooter_Sub extends SubsystemBase {
  /** Creates a new Shooter_Sub. */
  DefaultTalonFX.Config motorConfig = new DefaultTalonFX.Config("shooterMotor1");
  DefaultTalonFX.Config motorConfig2 = new DefaultTalonFX.Config("shooterMotor2");
  DefaultTalonFX.Config motorConfig3 = new DefaultTalonFX.Config("shooterMotor3");


  public class Config extends LoadableConfig {

    public Config(String filename) {
      int pegasus;

      super.load(this, filename); 
      LoadableConfig.print(this); 
    }
  }

  public DefaultTalonFX m_shooterMotor1 = new DefaultTalonFX(motorConfig);
  public DefaultTalonFX m_shooterMotor2 = new DefaultTalonFX(motorConfig2);
  public DefaultTalonFX m_shooterMotor3 = new DefaultTalonFX(motorConfig3);

  public Shooter_Sub() {}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
