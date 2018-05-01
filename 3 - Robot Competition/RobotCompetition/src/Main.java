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
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class Main {
	
			static MovePilot robotPilot;
			static RegulatedMotor frontNeckMotor = Motor.A;
			static EV3UltrasonicSensor distanceSensor;
			static EV3ColorSensor colorSensor;
			static EV3GyroSensor gyroSensor;
			static SampleProvider distanceSampleProvider;
			static SampleProvider colorSampleProvider;
			static SampleProvider gyroSampleProvider;

			public static Robot robotModel;
			public static World world;

			static long startTimeMillis;

			

		public static void main(String[] args) {
			// Sets up pilot and sensors for robot
			Wheel leftWheel = WheeledChassis.modelWheel(Motor.C, 5.65).offset(7.25);
			Wheel rightWheel = WheeledChassis.modelWheel(Motor.B, 5.65).offset(-7.25);
			Chassis myChassis = new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
			robotPilot = new MovePilot(myChassis);
			distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
			colorSensor = new EV3ColorSensor(SensorPort.S4);
			gyroSensor = new EV3GyroSensor(SensorPort.S2);
			distanceSampleProvider = distanceSensor.getDistanceMode();
			gyroSampleProvider = gyroSensor.getAngleMode();

			// feel free to play with these numbers...
			robotPilot.setLinearSpeed(30);
			robotPilot.setLinearAcceleration(40);
			robotPilot.setAngularSpeed(60);

			LCD.clear();
			LCD.drawString("ENTER to run!", 0, 0);
			Button.waitForAnyPress();
			LCD.clear();
			

			gyroSensor.reset(); // reset the gyroSensor

			boolean[] directions = lookAround();
			robotModel = new Robot();
			world = new World(directions[1], directions[2]);
			
			run();
		}

}

