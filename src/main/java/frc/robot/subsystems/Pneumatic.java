// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatic extends SubsystemBase {
  /** Creates a new Pneumatic. */
  public Pneumatic() {}
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0, 1);
  public void startCompressor(){
    compressor.enableDigital();;
  }

  public void push ()
  {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void pull ()
  {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void stop ()
  {
    solenoid.set(DoubleSolenoid.Value.kOff);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
