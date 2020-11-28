/*
 * Copyright (c) 2018-2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

public class CANDigitalInput {
  public enum LimitSwitch {
    kForward(0), kReverse(1);

    @SuppressWarnings("MemberName")
    public final int value;

    LimitSwitch(int value) {
      this.value = value;
    }
  }

  public enum LimitSwitchPolarity {
    kNormallyOpen(0), kNormallyClosed(1);

    @SuppressWarnings("MemberName")
    public final int value;

    LimitSwitchPolarity(int value) {
      this.value = value;
    }
  }

  private final CANSparkMax m_device;
  private LimitSwitch m_limitSwitch = LimitSwitch.kForward;

  /**
   * Constructs a CANDigitalInput.
   *
   * @param device      The Spark Max to which the limit switch is attached.
   * @param limitSwitch Whether this is forward or reverse limit switch.
   * @param polarity    Whether the limit switch is normally open or normally
   *                    closed.
   */
  public CANDigitalInput(CANSparkMax device, LimitSwitch limitSwitch,
                         LimitSwitchPolarity polarity) {
    m_device = device;
    m_limitSwitch = limitSwitch;
    
    if (m_device.m_altEncInitialized) {
      throw new IllegalArgumentException("Cannot instantiate a limit switch while using an alternative encoder");
    }
    
    CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetDataPortConfig(m_device.m_sparkMax, 0));
    CANSparkMaxJNI.c_SparkMax_SetLimitPolarity(m_device.m_sparkMax, limitSwitch.value, polarity.value);
  }

  /**
   * Get the value from a digital input channel.
   *
   * Retrieve the value of a single digital input channel from a motor
   * controller. This method will return the state of the limit input
   * based on the selected polarity, whether or not it is enabled.
   *
   * @return The state of the limit switch based on the configured polarity
   */
  public boolean get() {
    if (m_limitSwitch == LimitSwitch.kForward) {
      return m_device.getFault(CANSparkMax.FaultID.kHardLimitFwd);
    } else {
      return m_device.getFault(CANSparkMax.FaultID.kHardLimitRev);
    }
  }

  /**
   * Enables or disables controller shutdown based on limit switch.
   *
   * @param enable  Enable/disable motor shutdown based on limit switch
   * state. This does not effect the result of the get() command.
   *
   * @return CANError Set to CANError::kOk if successful
   *
   */
  public CANError enableLimitSwitch(boolean enable) {
    return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableLimitSwitch(m_device.m_sparkMax, m_limitSwitch.value, enable));
  }

  /**
   * @return True if limit switch is enabled
   */
  public boolean isLimitSwitchEnabled() {
    return CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(m_device.m_sparkMax, m_limitSwitch.value);
  }
}
