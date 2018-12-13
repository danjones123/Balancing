package Balancing;

import java.io.File;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * This class aims to keep the robot balanced This is done through reading in
 * angels and angle velocities from the Gyro sensor and the motors, using them
 * to calculate the motor output to keep it balanced.
 * 
 */

public class TheBalancingAct extends Thread {
	UnregulatedMotor rightMotor = new UnregulatedMotor(MotorPort.A);
	UnregulatedMotor leftMotor = new UnregulatedMotor(MotorPort.D);
	EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);

	double speed = 0;
	double direction = 0;

	public void limitSpeed(double LSpeed) {
		if (LSpeed > 5) {
			LSpeed = 5;
		}
		if (LSpeed < -10) {
			LSpeed = -5;
		}
		speed = LSpeed;
	}

	/*
	 * Every 10ms, this method will measure the speed of the gyro angle and the
	 * motor rotation speed Using this information, it will calculate the CURRENT
	 * gyro angle and rotation speed The new motor power is the computed
	 */
	public void run() {
		double gyroAngle = 0; // All the necessary variables needed to calculate how much power to send to the wheels
		double gyroSpeed = 0; // The most important ones being the angle and the rotational angle
		double motorPos = 0; // Measured in degrees per second.
		double motorSpeed = 0;
		double motorSum = 0;
		double motorDifference = 0; // Motor differences, four are used to calc an avg.
		double mDP1 = 0;
		double mDP2 = 0;
		double mDP3 = 0;
		double power = 0;
		boolean ready = false;
		int loopCount = 0;

		rightMotor.resetTachoCount(); // Recalibrate the motors
		leftMotor.resetTachoCount();
		gyroSensor.reset(); // Recalibrate the gyro (Reset)
		SampleProvider gyroRead = gyroSensor.getRateMode(); // Measures the angular velocity of the Gyro.
		float[] sample = new float[gyroRead.sampleSize()]; // Sample represents the angular velocity in degrees per
															// second
		long lastTimeStep = System.nanoTime(); // Returns the value of running JVM in nano-seconds.
		LCD.drawString(("D/S" + sample[0]), 2, 1);

		gyroAngle = -0.25;
		Sound.beepSequenceUp(); // Sound to let the user know the robot has started up.
		Thread.currentThread().setPriority(MAX_PRIORITY); // Sets the priority of this thread to maximum.

		// Feedback loop to make the robot balance, press up button to make the robot
		// quit.
		while (!Button.UP.isDown()) {
			long now = System.nanoTime();
			double delta_time = (now - lastTimeStep) / 1000000000.0; // Time step in seconds
			lastTimeStep = now;

			// Obtain angle and speed
			gyroSensor.fetchSample(sample, 0);
			gyroSpeed = -sample[0];
			gyroAngle = gyroAngle + (gyroSpeed * delta_time); // Calculates the angular velocity (angular velocity
																// integrated over time)
			//LCD.drawString("D/S:" + sample[0], 2, 2);
			// Obtain motor rotation and the speed of rotation
			double motorSumOld = motorSum;
			double rightTacho = rightMotor.getTachoCount(); // Tacho count in degrees
			double leftTacho = leftMotor.getTachoCount();
			motorSum = rightTacho + leftTacho;
			//LCD.drawString("motorSum:" + motorSum, 2, 3);
			motorDifference = motorSum - motorSumOld;
			//LCD.drawString("Motor Difference:" + motorDifference, 2, 2);
			motorPos = motorPos + motorDifference;
			//LCD.drawString("motorPos:" + motorPos, 2, 4);
			motorSpeed = ((motorDifference + mDP1 + mDP2 + mDP3) / 4.0) / delta_time; // RotationalSpeed
			//LCD.drawString("motorSpeed:" + motorSpeed, 2, 5);
			mDP3 = mDP2;
			//LCD.drawString("mDP3:" + mDP3, 2, 2);
			mDP2 = mDP1;
			mDP1 = motorDifference;

			// Compute new motor power
			final double MAGIC_NUMBER_1 = 0.1; //0.09; // Giving weight to the variables, specific to the
			final double MAGIC_NUMBER_2 = 0.12; //0.12; // GyroBoy model (LEGOMINDSTORMS WEBSITE)
			final double MAGIC_NUMBER_3 = 1.1; //1.1; // Values such as gyro height its angle
			final double MAGIC_NUMBER_4 = 16; //16;

			motorPos -= speed;
			power = MAGIC_NUMBER_1 * motorSpeed + MAGIC_NUMBER_2 * motorPos + MAGIC_NUMBER_3 * gyroSpeed
					+ MAGIC_NUMBER_4 * gyroAngle;
			if (power > 100) {
				power = 100;
			}
			if (power < -100) {
				power = -100;
			}
			if (ready) {
				rightMotor.setPower((int) (power - direction));
				leftMotor.setPower((int) (power + direction));
			}
			Delay.msDelay(10);
			loopCount++;
			if (loopCount == 10) {
				ready = true;
			}
		}
	}
		
		public void fallen() {
			

			EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S4);
			SampleProvider sp = us.getDistanceMode();
			float [] samples = new float[1];
		
			sp.fetchSample(samples, 0);
		
			File ivefallen = new File("iveFallen2 .wav");
		
			while(true) {
				if(samples[0] > 0.07) {
					rightMotor.stop();
					leftMotor.stop();
		
					LCD.clear ();
					Delay.msDelay(500);
					
					LCD.drawString ( " I've fallen " , 2 , 2);
					LCD.drawString(" and I can't ", 2, 3);
					LCD.drawString(" get up ", 2, 4);
			
					Sound.playSample(ivefallen, 100);
			
					Delay.msDelay(4000);
					rightMotor.close();
					leftMotor.close();
					gyroSensor.close();
					us.close();
					System.exit(0);
				}
					sp.fetchSample(samples, 0);
			}
		}
		
		//rightMotor.close();
		//leftMotor.close();
		//gyroSensor.close();
	}
