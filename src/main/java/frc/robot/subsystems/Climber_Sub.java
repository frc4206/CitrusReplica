// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;
import frc.robot.common.LoadableConfig;

public class Climber_Sub extends SubsystemBase {
  /** Creates a new Climber_Sub. */
  DefaultTalonFX.Config Mallard = new DefaultTalonFX.Config("Canadain Goose");

  
  public class  Config  extends LoadableConfig {
    public Config(String filename){
      int pekin;

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX m_climberMotor = new DefaultTalonFX(70,Mallard);


  public Climber_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
