/*
 * Copyright (c) 2018-2019 REV Robotics
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

import java.util.concurrent.atomic.AtomicBoolean;

import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

public class CANEncoder extends CANSensor {
    private int m_counts_per_rev = 4096;
	
    private AtomicBoolean m_encInitialized = new AtomicBoolean(false);
    private AtomicBoolean m_altEncInitialized = new AtomicBoolean(false);

    /**
     * Constructs a CANEncoder.
     *
     * @param device The Spark Max to which the encoder is attached.
     * @param sensorType The encoder type for the motor: kHallEffect or kQuadrature
     * @param counts_per_rev The counts per revolution of the encoder
     */
    public CANEncoder(CANSparkMax device, EncoderType sensorType, int counts_per_rev) {
      super(device);
      if (!m_encInitialized.get() || m_counts_per_rev != counts_per_rev) { 
        m_encInitialized.set(true);
        m_altEncInitialized.set(false);
        m_counts_per_rev = counts_per_rev;
        CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSensorType(m_device.m_sparkMax, sensorType.value));
        if (!(sensorType == EncoderType.kHallSensor || m_counts_per_rev == 0)) {
          CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetCountsPerRevolution(m_device.m_sparkMax, counts_per_rev));
        }
      }
    }
    
    /**
     * Constructs a CANEncoder.
     *
     * @param device The Spark Max to which the encoder is attached.
     * @param sensorType The encoder type for the motor: kHallEffect or kQuadrature
     * @param counts_per_rev The counts per revolution of the encoder
     */
    public CANEncoder(CANSparkMax device, AlternateEncoderType sensorType, int counts_per_rev) {
      super(device);
      if (m_device.m_limitSwitchInitialized) {
        throw new IllegalArgumentException("Cannot instantiate an alternative encoder while limit switches are enabled");
      }

      if(!m_altEncInitialized.get()) {
        m_altEncInitialized.set(true);
        m_encInitialized.set(false);
        CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetDataPortConfig(m_device.m_sparkMax, 1));
      }

      if(m_counts_per_rev != counts_per_rev) {
        m_counts_per_rev = counts_per_rev;
        CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderCountsPerRevolution(m_device.m_sparkMax, counts_per_rev));
      }
    }

    /**
     * Constructs a CANEncoder.
     *
     * @param device The Spark Max to which the encoder is attached.
     */
    public CANEncoder(CANSparkMax device) {
      this(device, EncoderType.kHallSensor, 42);
    }

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     *
     */
    public double getPosition() {
      if (m_encInitialized.get()) {
        return (double)CANSparkMaxJNI.c_SparkMax_GetEncoderPosition(m_device.m_sparkMax);
      } else {
        return (double)CANSparkMaxJNI.c_SparkMax_GetAltEncoderPosition(m_device.m_sparkMax);
      }
      
    }

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'RPM' by default, and can be changed by a scale factor
     * using setVelocityConversionFactor().
     *
     * @return Number the RPM of the motor
     *
     */
    public double getVelocity() {
      if (m_encInitialized.get()) {
        return (double)CANSparkMaxJNI.c_SparkMax_GetEncoderVelocity(m_device.m_sparkMax);
      } else {
        return (double)CANSparkMaxJNI.c_SparkMax_GetAltEncoderVelocity(m_device.m_sparkMax);
      }
      
    }

    /**
     * Set the position of the encoder.  By default the units
     * are 'rotations' and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @param position Number of rotations of the motor
     *
     * @return CANError Set to CANError.kOK if successful
     */
    public CANError setPosition(double position) {
      if (m_encInitialized.get()) {
        return m_device.setEncPosition(position);
      } else {
        return m_device.setAltEncPosition(position);
      }
      
    }

    /**
     * Set the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position.
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return CANError Set to CANError.kOK if successful
     */
    public CANError setPositionConversionFactor(double factor) {
      if (m_encInitialized.get()) {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetPositionConversionFactor(m_device.m_sparkMax, (float)factor));
      } else {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderPositionFactor(m_device.m_sparkMax, (float)factor));
      }
      
    }

    /**
     * Set the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return CANError Set to CANError.kOK if successful
     */
    public CANError setVelocityConversionFactor(double factor) {
      if (m_encInitialized.get()) {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetVelocityConversionFactor(m_device.m_sparkMax, (float)factor));
      } else {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderVelocityFactor(m_device.m_sparkMax, (float)factor));
      }
      
    }

    /**
     * Get the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position
     *
     * @return The conversion factor for position
     */
    public double getPositionConversionFactor() {
      if (m_encInitialized.get()) {
        return (double)CANSparkMaxJNI.c_SparkMax_GetPositionConversionFactor(m_device.m_sparkMax);
      } else {
        return (double)CANSparkMaxJNI.c_SparkMax_GetAltEncoderPositionFactor(m_device.m_sparkMax);
      }
      
    }

    /**
     * Get the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @return The conversion factor for velocity
     */
    public double getVelocityConversionFactor() {
      if (m_encInitialized.get()) {
        return (double)CANSparkMaxJNI.c_SparkMax_GetVelocityConversionFactor(m_device.m_sparkMax);
      } else {
        return (double)CANSparkMaxJNI.c_SparkMax_GetAltEncoderVelocityFactor(m_device.m_sparkMax);
      }
      
    }
    
    /**
     * Set the average sampling depth for a quadrature encoder. This value
     * sets the number of samples in the average for velocity readings. This
     * can be any value from 1 to 64.
     * 
     * When the SparkMax controller is in Brushless mode, this 
     * will not change any behavior.
     * 
     * @param depth The average sampling depth between 1 and 64 (default)
     * 
     * @return CANError.kOK if successful
     */
    public CANError setAverageDepth(int depth) {
      if (m_encInitialized.get()) {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAverageDepth(m_device.m_sparkMax, depth));
      } else {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderAverageDepth(m_device.m_sparkMax, depth));
      }
      
    }

    /**
     * Get the averafe sampling depth for a quadrature encoder. 
     * 
     * @return The average sampling depth
     */
    public int getAverageDepth() {
      if (m_encInitialized.get()) {
        return (int)CANSparkMaxJNI.c_SparkMax_GetAverageDepth(m_device.m_sparkMax);
      } else {
        return (int)CANSparkMaxJNI.c_SparkMax_GetAltEncoderAverageDepth(m_device.m_sparkMax);
      }
      
    }

    /**
     * Set the measurement period for velocity measurements of a quadrature encoder.
     * When the SparkMax controller is in Brushless mode, this will not
     * change any behavior.
     * 
     * The basic formula to calculate velocity is change in positon / change in time.
     * This parameter sets the change in time for measurement.
     * 
     * @param period_us Measurement period in milliseconds. This number may be
     * between 1 and 100 (default).
     * 
     * @return CANError.kOK if successful
     */
    public CANError setMeasurementPeriod(int period_us) {
      if (m_encInitialized.get()) {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetMeasurementPeriod(m_device.m_sparkMax, period_us));
      } else {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderMeasurementPeriod(m_device.m_sparkMax, period_us));
      }
      
    }

    /**
     * Get the number of samples for reading from a quadrature encoder.
     * 
     * @return Number of samples
     */ 
    public int getMeasurementPeriod() {
      if (m_encInitialized.get()) {
        return (int)CANSparkMaxJNI.c_SparkMax_GetMeasurementPeriod(m_device.m_sparkMax);
      } else {
        return (int)CANSparkMaxJNI.c_SparkMax_GetAltEncoderMeasurementPeriod(m_device.m_sparkMax);
      }
      
    }

    /**
     * Get the counts per revolution of the quadrature encoder. 
     * 
     * For a description on the difference between CPR, PPR, etc. go to
     * https://www.cuidevices.com/blog/what-is-encoder-ppr-cpr-and-lpr
     * 
     * @deprecated
     * This method is not clear about what it returns without reading the docs, 
     * <p> Use {@link CANEncoder#getCountsPerRevolution()} instead.
     * 
     * @return Counts per revolution
     */
    @Deprecated(since = "1.5.0", forRemoval = true)
    public int getCPR() {
      return getCountsPerRevolution();
    }

    /**
     * Get the counts per revolution of the quadrature encoder. 
     * 
     * For a description on the difference between CPR, PPR, etc. go to
     * https://www.cuidevices.com/blog/what-is-encoder-ppr-cpr-and-lpr
     * 
     * @return Counts per revolution
     */
    public int getCountsPerRevolution() {
      if (m_encInitialized.get()) {
        return (int)CANSparkMaxJNI.c_SparkMax_GetCountsPerRevolution(m_device.m_sparkMax);
      } else {
        return (int)CANSparkMaxJNI.c_SparkMax_GetAltEncoderCountsPerRevolution(m_device.m_sparkMax);
      }
      
    }

    @Override
	  protected int getID() {
      if (m_encInitialized.get()) {
        return (m_device.getInitialMotorType() == MotorType.kBrushless) 
                ? FeedbackSensorType.kHallSensor.value : FeedbackSensorType.kQuadrature.value;
      } else {
        return FeedbackSensorType.kAltQuadrature.value;
      }
      
    }
    
    @Override
    public CANError setInverted(boolean inverted) {
      if (m_encInitialized.get()) {
        if (m_device.getInitialMotorType() == MotorType.kBrushless) {
          throw new IllegalArgumentException("Not available in Brushless Mode");
        }
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetEncoderInverted(m_device.m_sparkMax, inverted));
      } else {
        return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetAltEncoderInverted(m_device.m_sparkMax, inverted));
      }
    }

    @Override
    public boolean getInverted() {
      if (m_encInitialized.get()) {
        return (boolean)CANSparkMaxJNI.c_SparkMax_GetEncoderInverted(m_device.m_sparkMax);
      } else {
        return (boolean)CANSparkMaxJNI.c_SparkMax_GetAltEncoderInverted(m_device.m_sparkMax);
      }
    }
}
