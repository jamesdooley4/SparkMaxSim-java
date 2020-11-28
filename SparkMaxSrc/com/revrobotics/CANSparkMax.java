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

import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.jni.CANSparkMaxJNI;

public class CANSparkMax extends CANSparkMaxLowLevel implements AutoCloseable {
	
	public enum IdleMode {
		kCoast(0), kBrake(1);

		@SuppressWarnings("MemberName")
		public final int value;

		IdleMode(int value) {
			this.value = value;
		}

		public static IdleMode fromId(int id) {
			if (id == 1) {
				return kBrake;
			}
			return kCoast;
		}
	}

	public enum InputMode {
		kPWM(0), kCAN(1);

		@SuppressWarnings("MemberName")
		public final int value;

		InputMode(int value) {
			this.value = value;
		}

		public static InputMode fromId(int id) {
			if (id == 1) {
				return kCAN;
			}
			return kPWM;
		}
	}

	public enum SoftLimitDirection {
		kForward(0), kReverse(1);

		@SuppressWarnings("MemberName")
		public final int value;

		SoftLimitDirection(int value) {
			this.value = value;
		}

		public static SoftLimitDirection fromID(int id) {
			if (id == 1) {
				return kReverse;
			}
			return kForward;
		}
	}

	public enum FaultID {
		kBrownout(0), kOvercurrent(1), kIWDTReset(2), kMotorFault(3), kSensorFault(4), kStall(5), kEEPROMCRC(6),
		kCANTX(7), kCANRX(8), kHasReset(9), kDRVFault(10), kOtherFault(11), kSoftLimitFwd(12), kSoftLimitRev(13),
		kHardLimitFwd(14), kHardLimitRev(15);

		@SuppressWarnings("MemberName")
		public final int value;

		FaultID(int value) {
			this.value = value;
		}

		public static FaultID fromId(int id) {
			for (FaultID type : values()) {
				if (type.value == id) {
					return type;
				}
			}
			return null;
		}
	}

	public static class ExternalFollower {
		private final int arbId;
		private final int configId;

		public static final ExternalFollower kFollowerDisabled = new ExternalFollower(0, 0);
		public static final ExternalFollower kFollowerSparkMax = new ExternalFollower(0x2051800, 26);
		public static final ExternalFollower kFollowerPhoenix = new ExternalFollower(0x2040080, 27);

		public ExternalFollower(int arbId, int configId) {
			this.arbId = arbId;
			this.configId = configId;
		}
	}

	boolean m_altEncInitialized = false;
	boolean m_limitSwitchInitialized = false;
	

	/**
	 * Create a new SPARK MAX Controller
	 *
	 * @param deviceID The device ID.
	 * @param type     The motor type connected to the controller. Brushless motors
	 *                 must be connected to their matching color and the hall sensor
	 *                 plugged in. Brushed motors must be connected to the Red and
	 *                 Black terminals only.
	 */
	public CANSparkMax(int deviceID, MotorType type) {
		super(deviceID, type);

	}

	/**
	 * Closes the SPARK MAX Controller
	 */
	@Override
	public void close() {
	}

	/**** Speed Controller Interface ****/
	/**
	 * Common interface for setting the speed of a speed controller.
	 *
	 * @param speed The speed to set. Value should be between -1.0 and 1.0.
	 */
	@Override
	public void set(double speed) {
		setpointCommand(speed, ControlType.kDutyCycle);
	}

    /**
     * Sets the voltage output of the SpeedController.  This is equivillant to 
     * a call to SetReference(output, rev::ControlType::kVoltage). The behavior
     * of this call differs slightly from the WPILib documetation for this call
     * since the device internally sets the desired voltage (not a compensation value).
     * That means that this *can* be a 'set-and-forget' call.
     *
     * @param outputVolts The voltage to output.
     */
	@Override
	public void setVoltage(double outputVolts) {
		setpointCommand(outputVolts, ControlType.kVoltage);
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() {
		return getAppliedOutput();
	}

	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * This call has no effect if the controller is a follower.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	@Override
	public void setInverted(boolean isInverted) {
		CANSparkMaxJNI.c_SparkMax_SetInverted(m_sparkMax, isInverted);
	}

	/**
	 * Common interface for returning the inversion state of a speed controller.
	 * 
     * This call has no effect if the controller is a follower.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		return CANSparkMaxJNI.c_SparkMax_GetInverted(m_sparkMax);
	}

	/**
	 * Common interface for disabling a motor.
	 */
	@Override
	public void disable() {
		set(0);
	}

	@Override
	public void stopMotor() {
		set(0);
	}

	@Override
	public void pidWrite(double output) {
		set(output);
	}

	/******* Extended Functions *******/
	/**
	 * Returns and object for interfacing with the encoder connected to the 
	 * encoder pins or front port of the SPARK MAX.
	 * 
	 * The default encoder type is assumed to be the hall effect for brushless.
	 * This can be modified for brushed DC to use a quadrature encoder.
	 * 
	 * Assumes that the encoder the is integrated encoder, configured as:
	 * EncoderType.kHallEffect, 0 counts per revolution.
	 * 
	 * @return An object for interfacing with the integrated encoder.
	 */
	public CANEncoder getEncoder() {
		return getEncoder(EncoderType.kHallSensor, 0);
	}

	/**
	 * Returns and object for interfacing with the encoder connected to the 
	 * encoder pins or front port of the SPARK MAX.
	 * 
	 * The default encoder type is assumed to be the hall effect for brushless.
	 * This can be modified for brushed DC to use a quadrature encoder.
	 * 
	 * @param sensorType The encoder type for the motor: kHallEffect or kQuadrature
	 * @param counts_per_rev The counts per revolution of the encoder
	 * @return An object for interfacing with an encoder
	 */
	public CANEncoder getEncoder(EncoderType sensorType, int counts_per_rev) {
		return new CANEncoder(this, sensorType, counts_per_rev);
	}

	/**
	 * Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins. This is defined as :
	 * 
	 * Mutli-function Pin: Encoder A
	 * Limit Switch Reverse: Encoder B
	 * 
	 * This call will disable the limit switch inputs
	 * @return Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins
	 */
	public CANEncoder getAlternateEncoder() {
		return new CANEncoder(this, AlternateEncoderType.kQuadrature, 0);
	}

	/**
	 * Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins. This is defined as :
	 * 
	 * Mutli-function Pin: Encoder A
	 * Limit Switch Reverse: Encoder B
	 * 
	 * This call will disable the limit switch inputs.
	 * 
	 * @param counts_per_rev the Counts per revolution of the encoder
	 * @param sensorType The encoder type for the motor: currently only kQuadrature
	 * @return Returns an object for interfacing with an encoder connected to the alternate 
	 * data port configured pins
	 */
	public CANEncoder getAlternateEncoder(AlternateEncoderType sensorType, int counts_per_rev) {
		return new CANEncoder(this, sensorType, counts_per_rev);
	}

	/**
	 * @param mode The mode of the analog sensor, either absolute or relative
	 * @return An object for interfacing with a connected analog sensor.
	 */
	public CANAnalog getAnalog(AnalogMode mode) {
		return new CANAnalog(this, mode);
	}

	/**
	 * @return An object for interfacing with the integrated PID controller.
	 */
	public CANPIDController getPIDController() {
		return new CANPIDController(this);
	}

	/**
	 * @return An object for interfacing with the integrated forward limit switch.
	 *
	 * @param polarity Whether the limit switch is normally open or normally closed.
	 */
	public CANDigitalInput getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
		return new CANDigitalInput(this, CANDigitalInput.LimitSwitch.kForward, polarity);
	}

	/**
	 * @return An object for interfacing with the integrated reverse limit switch.
	 *
	 * @param polarity Whether the limit switch is normally open or normally closed.
	 */
	public CANDigitalInput getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
		return new CANDigitalInput(this, CANDigitalInput.LimitSwitch.kReverse, polarity);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 *
	 */
	public CANError setSmartCurrentLimit(int limit) {
		return setSmartCurrentLimit(limit, 0, 20000);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setSmartCurrentLimit(int stallLimit, int freeLimit) {
		return setSmartCurrentLimit(stallLimit, freeLimit, 20000);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 * @param limitRPM   RPM less than this value will be set to the stallLimit, RPM
	 *                   values greater than limitRPM will scale linearly to
	 *                   freeLimit
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSmartCurrentLimit(m_sparkMax, stallLimit, freeLimit, limitRPM));
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setSecondaryCurrentLimit(double limit) {
		return setSecondaryCurrentLimit(limit, 0);
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit      The current limit in Amps.
	 * @param chopCycles The number of additional PWM cycles to turn the driver off
	 *                   after overcurrent is detected.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setSecondaryCurrentLimit(double limit, int chopCycles) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSecondaryCurrentLimit(m_sparkMax, (float)limit, chopCycles));
	}

	/**
	 * Sets the idle mode setting for the SPARK MAX.
	 *
	 * @param mode Idle mode (coast or brake).
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setIdleMode(IdleMode mode) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetIdleMode(m_sparkMax, mode.value));
	}

	/**
	 * Gets the idle mode setting for the SPARK MAX.
	 *
	 * This uses the Get Parameter API and should be used infrequently. This
	 * function uses a non-blocking call and will return a cached value if the
	 * parameter is not returned by the timeout. The timeout can be changed by
	 * calling SetCANTimeout(int milliseconds)
	 *
	 * @return IdleMode Idle mode setting
	 */
	public IdleMode getIdleMode() {
		return IdleMode.fromId(CANSparkMaxJNI.c_SparkMax_GetIdleMode(m_sparkMax));
	}

	/**
	 * Sets the voltage compensation setting for all modes on the SPARK MAX and
	 * enables voltage compensation.
	 *
	 * @param nominalVoltage Nominal voltage to compensate output to
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError enableVoltageCompensation(double nominalVoltage) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableVoltageCompensation(m_sparkMax, (float)nominalVoltage));
	}

	/**
	 * Disables the voltage compensation setting for all modes on the SPARK MAX.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError disableVoltageCompensation() {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_DisableVoltageCompensation(m_sparkMax));
	}

	/**
	 * Get the configured voltage compensation nominal voltage value
	 *
	 * @return The nominal voltage for voltage compensation mode.
	 */
	public double getVoltageCompensationNominalVoltage() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetVoltageCompensationNominalVoltage(m_sparkMax);
	}

	/**
	 * Sets the ramp rate for open loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setOpenLoopRampRate(double rate) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetOpenLoopRampRate(m_sparkMax, (float)rate));
	}

	/**
	 * Sets the ramp rate for closed loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setClosedLoopRampRate(double rate) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetClosedLoopRampRate(m_sparkMax, (float)rate));
	}

	/**
	 * Get the configured open loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getOpenLoopRampRate() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetOpenLoopRampRate(m_sparkMax);
	}

	/**
	 * Get the configured closed loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getClosedLoopRampRate() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetClosedLoopRampRate(m_sparkMax);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
     * 
     * The motor will spin in the same direction as the leader. This can be changed
     * by passing a true constant after the leader parameter.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader The motor controller to follow.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError follow(final CANSparkMax leader) {
		return follow(leader, false);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader The motor controller to follow.
	 * @param invert Set the follower to output opposite of the leader
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError follow(final CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSparkMax, leader.getDeviceId(), invert);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
	 * The motor will spin in the same direction as the leader. This can be changed
	 * by passing a true constant after the deviceID parameter.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError follow(ExternalFollower leader, int deviceID) {
		boolean inverted = CANSparkMaxJNI.c_SparkMax_GetInverted(m_sparkMax);
		return follow(leader, deviceID, inverted);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 * 
     * Following anything other than a CAN SPARK MAX is not officially supported.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @param invert   Set the follower to output opposite of the leader
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError follow(ExternalFollower leader, int deviceID, boolean invert) {
		FollowConfig maxFollower = new FollowConfig();
		maxFollower.leaderArbId = (leader.arbId == 0 ? 0 : leader.arbId | deviceID);
		maxFollower.config.predefined = leader.configId;
		maxFollower.config.invert = invert ? 1 : 0;
		return setFollow(maxFollower);
	}

	/**
	 * Returns whether the controller is following another controller
	 *
	 * @return True if this device is following another controller false otherwise
	 */
	public boolean isFollower() {
		return CANSparkMaxJNI.c_SparkMax_IsFollower(m_sparkMax);
	}

	/**
	 * @return All fault bits as a short
	 */
	public short getFaults() {
		return (short)CANSparkMaxJNI.c_SparkMax_GetFaults(m_sparkMax);
	}

	/**
	 * @return All sticky fault bits as a short
	 */
	public short getStickyFaults() {
		return (short)CANSparkMaxJNI.c_SparkMax_GetStickyFaults(m_sparkMax);
	}

	/**
	 * Get the value of a specific fault
	 *
	 * @param faultID The ID of the fault to retrive
	 *
	 * @return True if the fault with the given ID occurred.
	 */
	public boolean getFault(FaultID faultID) {
		return CANSparkMaxJNI.c_SparkMax_GetFault(m_sparkMax, faultID.value);
	}

	/**
	 * Get the value of a specific sticky fault
	 *
	 * @param faultID The ID of the sticky fault to retrive
	 *
	 * @return True if the sticky fault with the given ID occurred.
	 */
	public boolean getStickyFault(FaultID faultID) {
		return CANSparkMaxJNI.c_SparkMax_GetStickyFault(m_sparkMax, faultID.value);
	}

	/**
	 * @return The voltage fed into the motor controller.
	 */
	public double getBusVoltage() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetBusVoltage(m_sparkMax);
	}

	/**
	 * @return The motor controller's applied output duty cycle.
	 */
	public double getAppliedOutput() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetAppliedOutput(m_sparkMax);
	}

	/**
	 * @return The motor controller's output current in Amps.
	 */
	public double getOutputCurrent() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetOutputCurrent(m_sparkMax);
	}

	/**
	 * @return The motor temperature in Celsius.
	 */
	public double getMotorTemperature() {
		return (double)CANSparkMaxJNI.c_SparkMax_GetMotorTemperature(m_sparkMax);
	}

	/**
	 * Clears all sticky faults.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError clearFaults() {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_ClearFaults(m_sparkMax));
	}

	/**
	 * Writes all settings to flash.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError burnFlash() {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_BurnFlash(m_sparkMax));
	}

	/**
	 * Sets timeout for sending CAN messages with SetParameter* and GetParameter* calls.
	 * These calls will block for up to this amoutn of time before returning a timeout
	 * erro. A timeout of 0 will make the SetParameter* calls non-blocking, and instead
	 * will check the response in a separate thread. With this configuration, any
	 * error messages will appear on the drivestration but will not be returned by the
	 * GetLastError() call.
	 *
	 * @param milliseconds The timeout in milliseconds.
	 *
	 * @return CANError Set to CANError.kOK if successful
	 */
	public CANError setCANTimeout(int milliseconds) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetCANTimeout(m_sparkMax, milliseconds));
	}

	/**
     * Enable soft limits
     *
     * @param direction the direction of motion to restrict
     * 
     * @param enable set true to enable soft limits
	 * 
	 * @return CANError Set to CANError.kOK if successful
     */
	public CANError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_EnableSoftLimit(m_sparkMax, direction.value, enable));
	}

	 /**
     * Set the soft limit based on position. The default unit is
     * rotations, but will match the unit scaling set by the user.
     * 
     * Note that this value is not scaled internally so care must
     * be taken to make sure these units match the desired conversion
     *
     * @param direction the direction of motion to restrict
     * 
     * @param limit position soft limit of the controller
	 * 
	 * @return CANError Set to CANError.kOK if successful
     */
	public CANError setSoftLimit(SoftLimitDirection direction, float limit) {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_SetSoftLimit(m_sparkMax, direction.value, limit));
	}

	/**
     * Get the soft limit setting in the controller
     *
     * @param direction the direction of motion to restrict
     * 
     * @return position soft limit setting of the controller
     */
	public double getSoftLimit(SoftLimitDirection direction) {
		return (double)CANSparkMaxJNI.c_SparkMax_GetSoftLimit(m_sparkMax, direction.value);
	}

	/**
	 * @param direction The direction of the motion to restrict
	 * 
     * @return true if the soft limit is enabled.
     */
	public boolean isSoftLimitEnabled(SoftLimitDirection direction) {
		return (boolean)CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(m_sparkMax, direction.value);
	}

	/**
	 * Gets the feedback device ID that was set on SparkMax itself.
	 * 
	 * @return Feedback device ID on the SparkMax
	 */
	protected int getFeedbackDeviceID() {
		return (int)CANSparkMaxJNI.c_SparkMax_GetFeedbackDeviceID(m_sparkMax);
	}

	/**
     * All device errors are tracked on a per thread basis for all
     * devices in that thread. This is meant to be called 
     * immediately following another call that has the possibility 
     * of returning an error to validate if an  error has occurred. 
     * 
     * @return the last error that was generated.
     */
	public CANError getLastError() {
		return CANError.fromInt(CANSparkMaxJNI.c_SparkMax_GetLastError(m_sparkMax));
	}
}
