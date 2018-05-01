
import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;


import java.util.Random;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.sensor.EV3TouchSensor;

public class Main {

	static MovePilot robot;
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;
	static EV3TouchSensor bumpSensorLeft;
	static EV3TouchSensor bumpSensorRight;
	static EV3ColorSensor colorSensor;
	
	
	public static void main(String[] args){
		Wheel leftWheel = WheeledChassis.modelWheel(leftMotor, 5.5).offset(7.25);
		Wheel rightWheel = WheeledChassis.modelWheel(rightMotor, 5.5).offset(-7.25);
		Chassis myChassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		robot= new MovePilot(myChassis);
		robot.setLinearSpeed(20);
		robot.setAngularSpeed(40);
		
		bumpSensorLeft = new EV3TouchSensor(SensorPort.S3);
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		bumpSensorRight = new EV3TouchSensor(SensorPort.S1);
		
		LCD.clear();
		LCD.drawString("Let's Push Cans!", 0,0);
		Button.waitForAnyPress();
		LCD.clear();
		
		run();
	}
	
	public static void run(){
	
		while(!Button.ESCAPE.isDown()){
			robot.rotate(70); //Turn Left
			moveTillWallHit(); //Hit wall
			curveRight(); //turn back toward cans
			moveTillWallHit(); //move until hit cans
			canTurn(); //can turn
			robot.travel(15); //half way  
			canTurn();
			moveTillWallHit();
			dropOff();
		}
	
	}
	
	public static void curveRight(){
		robot.travel(-6);
		robot.rotate(-15);
		robot.travel(3);
		robot.rotate(-15);
		robot.travel(3);
		robot.rotate(-15);
		robot.travel(3);
		robot.rotate(-7);
		robot.travel(4);
	}
	
	public static void moveTillWallHit(){
		SampleProvider bumpSampleLeftProvider = bumpSensorLeft.getTouchMode();
		float[] bumpSensorLeftData = new float[bumpSampleLeftProvider.sampleSize()];
		bumpSampleLeftProvider.fetchSample(bumpSensorLeftData, 0);
		
		SampleProvider bumpSensorRightProvider = bumpSensorRight.getTouchMode();
		float[] bumpSensorRightData = new float[bumpSensorRightProvider.sampleSize()];
		bumpSensorRightProvider.fetchSample(bumpSensorRightData, 0);
		
		robot.forward(); // moves robot forward until robot.stop() is called
		
		
		while((bumpSensorLeftData[0] != 1 && bumpSensorRightData[0] != 1) &&!Button.ESCAPE.isDown()){
//			robot.travel(-10); // back up
//			robot.rotate(90); // in degrees
			bumpSampleLeftProvider.fetchSample(bumpSensorLeftData, 0);
			bumpSensorRightProvider.fetchSample(bumpSensorRightData, 0);
			
		}
		robot.stop();	
	}
	
	public static void canTurn(){
		robot.travel(-6);
		robot.rotate(-20);
		robot.travel(6);
		robot.rotate(-20);
		robot.travel(6);
		robot.rotate(-20);
	}
	public static void dropOff(){
		robot.travel(-30);
	}

}
