//
//  main.c
//  211
//
//  Created by Yi on 18/9/23.
//  Copyright © 2018年 Fandi. All rights reserved.
//
/**
 * This class is meant as a skeleton for the odometer class to be used.
 *
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private double x, y, theta;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double previousTachoLeft;
	private double previousTachoRight;

	private double deltaTachoRight;
	private double deltaTachoLeft;

	private double angleChangeRight;
	private double angleChangeLeft;

	private double arcLengthLeft;
	private double arcLengthRight;
	private double arcCentre;

	private double deltaX;
	private double deltaY;
	private double deltaT;

	private double[] position;

	private static final long ODOMETER_PERIOD = 25; // odometer update period in
													// ms

	/**
	 * This is the default constructor of this class. It initiates all motors
	 * and variables once.It cannot be accessed externally.
	 *
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 *
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) {
			return odo;
		} else {
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant
	 * to be used only if an odometer object has been created
	 *
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		previousTachoLeft = leftMotor.getTachoCount();
		previousTachoRight = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			// leftMotor.resetTachoCount();
			// rightMotor.resetTachoCount();
			double[] information = odo.getXYT();

			leftMotorTachoCount = leftMotor.getTachoCount();
			rightMotorTachoCount = rightMotor.getTachoCount();

			deltaTachoRight = rightMotorTachoCount - previousTachoRight; // tachocount (degrees)														
			deltaTachoLeft = leftMotorTachoCount - previousTachoLeft;

			angleChangeLeft = deltaTachoLeft * 2 * Math.PI / 360; // convert to radians								
			angleChangeRight = deltaTachoRight * 2 * Math.PI / 360;

			previousTachoLeft = leftMotorTachoCount;
			previousTachoRight = rightMotorTachoCount;

			arcLengthRight = WHEEL_RAD * angleChangeRight; // in radians
			arcLengthLeft = WHEEL_RAD * angleChangeLeft;

			double arcCentre = (arcLengthRight + arcLengthLeft) / 2.0;
			double deltaT = (arcLengthRight - arcLengthLeft) / TRACK; // radians

			theta = information[2];
			theta = theta * 2 * Math.PI / 360; //convert to radians 
			theta += deltaT;

			deltaY = arcCentre * Math.cos(theta); //displacement in Y
			deltaX = arcCentre * Math.sin(theta); //displacement in X

			y += deltaY;
			x += deltaX;

			deltaT = 180 * deltaT / (Math.PI);
			theta = 180 * theta / (Math.PI);

			// TODO Calculate new robot position based on tachometer counts
			// (done)

			// TODO Update odometer values with new calculated values
			odo.update(deltaX, deltaY, deltaT); //updated values 

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();

			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

	// public void settheta (double theta) {
	// this.theta =theta;
	// }

	public double getdeltaT() {
		return deltaT;
	}
}
