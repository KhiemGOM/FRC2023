// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.nio.ByteBuffer;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lidar extends SubsystemBase {
  /** Creates a new Lidar. */

  private static final byte k_deviceAddress = 0x62;

	private final byte m_port;

	private final ByteBuffer m_buffer = ByteBuffer.allocateDirect(2);

	public Lidar(Port port) {
		m_port = (byte) port.value;
		I2CJNI.i2CInitialize(m_port);
	}

	public void startMeasuring() {
    //Use the delay value in MEASURE_DELAY (0x45) instead.
		writeRegister(0x04, 0x08 | 32); // default bits or'ed at 5th bit
    //Value 0xff will enable free running mode after the 
    //host device sends an initial measurement command.
		writeRegister(0x11, 0xff);
    //Take distance measurement with receiver bias correction
		writeRegister(0x00, 0x04);
	}

	public void stopMeasuring() {
    //One measurement per distance measurement command.
		writeRegister(0x11, 0x00);
	}
  /**
   * IDK how this work but if it works it works.
   * 
   * <p>The distance measurement is 16 bits long.
   * The first byte is the high byte and the second byte is the low byte.
   * @return The distance measurement in centimeters.
   */
	public int getDistance() {
		return readShort(0x8f);
	}

	private int writeRegister(int address, int value) {
		m_buffer.put(0, (byte) address);
		m_buffer.put(1, (byte) value);

		return I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 2);
	}

	private short readShort(int address) {
		m_buffer.put(0, (byte) address);
		I2CJNI.i2CWrite(m_port, k_deviceAddress, m_buffer, (byte) 1);
		I2CJNI.i2CRead(m_port, k_deviceAddress, m_buffer, (byte) 2);
		return m_buffer.getShort(0);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", getDistance());
  }
}
