// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DefaultTalonFX;

public class Intake_Sub extends SubsystemBase {
  /** Creates a new Intake_Sub. */
  public DefaultTalonFX m_intakeMotor = new DefaultTalonFX(40);
  
  public Intake_Sub() {
    m_intakeMotor.Enable_Sim();
    m_intakeMotor.Log_Data_To_Smart_Dashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intakeMotor.Update();
  }

  @Override
  public void simulationPeriodic() {
      super.simulationPeriodic();
      m_intakeMotor.Update_Sim();
      m_intakeMotor.Duty_Cycle(0.5, false);
  }
}
