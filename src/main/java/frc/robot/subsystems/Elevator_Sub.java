// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Elevator_Sub extends SubsystemBase {
  /** Creates a new Elevator_Sub. */
  public DefaultTalonFX m_elevatorMotor = new DefaultTalonFX(60);
  
  public Elevator_Sub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
