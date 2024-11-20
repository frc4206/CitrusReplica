// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;
import frc.robot.common.LoadableConfig;

public class Climber_Sub extends SubsystemBase {
  /** Creates a new Climber_Sub. */
  DefaultTalonFX.Config climberMotorConfig = new DefaultTalonFX.Config("ClimberMotorConfig");

  
  public class  Config  extends LoadableConfig {
    public double kHomePosition;
    public double kCruiseVelocity;
    public double kAcceleration;
    public double kMaxUnitsLimit;
    public double kMinUnitsLimit;
    public double kEnableSupplyCurrentLimit;
    public double kSupplyCurrentLimit;
    public double kSupplyCurrentThreshold;
    public double kSupplyCurrentTimeout;
    public double kMaxForwardOutput;
    public double kMaxReverseOutput;

    public Config(String filename){
      

      super.load(this, filename);
      LoadableConfig.print(this);
    }
  }

  public DefaultTalonFX m_climberMotor = new DefaultTalonFX(climberMotorConfig);


  public Climber_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
