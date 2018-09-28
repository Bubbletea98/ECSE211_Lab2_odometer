/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private static final int blackcolor = 300; // TODO:
	private static final double sensordistance = 0.9;
	private static final double tileLength = 30.48;

	public static double[] correctionData = new double[3];

	private int counterY;
	private int counterX;
	private static Port lsPort = LocalEV3.get().getPort("S2");
	private SensorModes lsSensor;
	private SampleProvider lsColour;
	private float[] lsData;
	private double tempX;
	private double tempY;
	private double deltaT;

	private int scaleUp = 1000; //provides comparable color value
	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		this.lsSensor = new EV3ColorSensor(lsPort);
		this.lsColour = lsSensor.getMode("Red");
		this.lsData = new float[lsSensor.sampleSize()];
		counterY = 0;
		counterX = 0;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		boolean lineFound = false; // boolean for black line detection
									

		final TextLCD t = LocalEV3.get().getTextLCD();
		
		while (true) {
			correctionStart = System.currentTimeMillis();

			double[] currPosition = odometer.getXYT();

			tempX = currPosition[0];
			tempY = currPosition[1];
			deltaT = currPosition[2];

			

			lsColour.fetchSample(lsData, 0); 
			float lightValue = lsData[0] * scaleUp; // scale up to compare values easily
			
			if (lightValue <= blackcolor && !lineFound) { //finds black line

				Sound.beep();
				

				if ((deltaT <= 290 && deltaT > 250) || (deltaT <= 120 && deltaT > 75)) { //checks for angles in X direction of movement
					counterX++;

					
					if (counterX == 1 || counterX == 6) {
						tempX = 0;
					} else if (counterX > 1 && counterX < 4) { //checks 2-3 tile lengths, ignores the first, allows us to eliminate error from starting in center and turning
						tempX = -(counterX - 1) * tileLength;
					} else if (counterX == 4) {
						
						tempX = -2 * tileLength;
					} else {
						
						tempX = -tileLength;
					}

					correctionData[0] = tempX;

				} else if ((deltaT >= 350 || deltaT < 10) || (deltaT >= 135 && deltaT < 200)) { //angles in Y plane of motion 
					counterY++;

					
					double offset = Math.cos(deltaT) * sensordistance; 

					
					if (counterY == 1 || counterY == 6) {
						tempY = 0;
					} else if (counterY > 1 && counterY < 4) { //checks 2-3 tiles lengths, helps acccomodate for extra half tile lengths while turning
						tempY = (counterY - 1) * tileLength;
					} else if (counterY == 4) {
						tempY = 2 * tileLength;
					} else {
						tempY = tileLength;
					}

					
					odometer.setY(tempY - offset);
					correctionData[1] = tempY;
				}
				lineFound = true;
			} else {
				lineFound = false;
			}

			correctionData[2] = deltaT;
			
			odometer.setXYT(tempX, tempY, deltaT); //set corrected X,Y,T

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
