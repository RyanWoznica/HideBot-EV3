import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SumoEyesSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.*;
import lejos.utility.Delay;
import lejos.*;

public class Test {
	static int maxSpeed = 300;
	static int defaultSpeed = 150;
	static int maxLight = 91;
	static int optimalLight = 20;
	public static void main(String[] args) throws InterruptedException {
		NXTLightSensor l1Sensor = new NXTLightSensor(SensorPort.S2);
		NXTLightSensor l2Sensor = new NXTLightSensor(SensorPort.S1);
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S3);
		SumoEyesSensor sumoSensor = new SumoEyesSensor(SensorPort.S4);
		int optimalSpaceCounterRight = 0;
		int optimalSpaceCounterLeft = 0;
		Sound.beepSequenceUp();
		/*
		 * Algorithm Steps:
		 * 	1) Determine if we're stuck via the touch sensor - if we are, back up!
		 * 	2) Determine if there's an obstacle in the way - if there is:
		 * 		2a) Determine the direction of movement that would best achieve the
		 * 			goal state (means-ends analysis) by comparing light sources
		 * 		2b) Set maximum speed to the wheel needed to move the robot towards
		 * 			the best direction
		 * 		2c) Keep track of which directions we've moved by incrementing
		 * 			optimalSpaceCounterRight/Left (more on this later)
		 * 	3) If we're clear, poll the light sensors and increase the speed on one
		 * 		of the motors to move towards the darkest possible route, but more
		 * 		gradually than we would in obstacle avoidance. Speed is actually
		 * 		dynamic, based on the difference in light intensities in each
		 * 		direction! The wheel not being accelerated then has its speed
		 * 		normalized (gradually decreased) towards the default speed.
		 * 		3a) Reset the optimalSpaceCounters since we're moving again...
		 * 	4) Check to see if we have the ideal goal states for having found a
		 * 		"hiding spot"! If the average detected brightness (of the two light
		 * 		sensors) is low enough, and we've turned around 3 times in one
		 * 		direction (indicating that we're somewhere cozy with lots of
		 * 		walls), we can then signal a victory condition and stop.
		 */
		while(true){
			// Beware! Motors are backwards on the robot, so they have to be
			// programmed in reverse!
			Motor.A.backward();
			Motor.B.backward();
			int obstacle = getObstacle(sumoSensor);
			int movement = getMovement(l1Sensor,l2Sensor);
			LCD.drawString(Integer.toString(obstacle), 0, 5);
			// Failsafe for collision detection, for objects like thin metal bars - 
			// just in case it gets stuck (as it's proven to before) >_>
			if(isBumped(touchSensor)){
				Motor.A.setSpeed(defaultSpeed);
				Motor.B.setSpeed(defaultSpeed);
				Motor.A.forward();
				Motor.B.forward();
				Thread.sleep(1500);
			}
			// If object detected, go in the direction of least light (means-ends
			// navigation) - stored in the movement decision
			if(obstacle > 1){
				if (movement > 0){
					optimalSpaceCounterRight++;
					Motor.A.setSpeed(defaultSpeed);
					Motor.B.setSpeed(maxSpeed);
					Thread.sleep(300);
				}
				else{
					optimalSpaceCounterLeft++;
					Motor.A.setSpeed(maxSpeed);
					Motor.B.setSpeed(defaultSpeed);
					Thread.sleep(300);
				}
			}
			else if(movement > 0){
				optimalSpaceCounterLeft = 0;
				optimalSpaceCounterRight = 0;
				Motor.A.setSpeed(defaultSpeed + (movement*5));
				normalizeSpeed(Motor.B);
			}
			else{
				optimalSpaceCounterLeft = 0;
				optimalSpaceCounterRight = 0;
				normalizeSpeed(Motor.A);
				Motor.B.setSpeed(defaultSpeed + Math.abs(movement*5));
			}
			if((optimalSpaceCounterLeft > 3 || optimalSpaceCounterRight > 3) && getAvgLight(l1Sensor, l2Sensor) <= optimalLight){
				Sound.beepSequence();
				Motor.A.stop();
				Motor.B.stop();
				break;
			}
		}
	}
	
	private static Boolean isBumped(EV3TouchSensor touchSensor) {
		SampleProvider result = touchSensor.getTouchMode();
		float[] sample = new float[result.sampleSize()];
		result.fetchSample(sample, 0);
		if (sample[0] == 1){
			return true;
		}
		return false;
	}

	private static int normalizeSpeed(NXTRegulatedMotor a) {
		if(a.getSpeed() > defaultSpeed + 10){
			a.setSpeed(a.getSpeed()-5);
		}
		else if (a.getSpeed() > defaultSpeed){
			a.setSpeed(defaultSpeed);
		}
		return 0;
	}

	// Returns 1 (no object - left (sensor defect)), 2 (object detected - middle),
	// or 3 (object detected - right)
	private static int getObstacle(SumoEyesSensor sumoSensor) {
		return sumoSensor.getObstacle();
	}

	// Returns: Integer representing the difference in light between the two directions.
	// If retval > 0, brightness is to the left, vice versa!
	static int getMovement(NXTLightSensor l1, NXTLightSensor l2){
		SensorMode l1Prov = l1.getAmbientMode();
		SensorMode l2Prov = l2.getAmbientMode();
		float[] sample = new float[l1.sampleSize() + l2.sampleSize()];
		l1Prov.fetchSample(sample, 0);
		l2Prov.fetchSample(sample, 1);
		LCD.drawString(Float.toString(sample[0]*100), 0, 1);
		LCD.drawString(Float.toString(sample[1]*100), 0, 2);
		return (int) (sample[0]*100 - sample[1]*100);
	}
	
	// Returns: Integer representing the average light intensity from two light
	// sensors.
	static int getAvgLight(NXTLightSensor l1, NXTLightSensor l2){
		SensorMode l1Prov = l1.getAmbientMode();
		SensorMode l2Prov = l2.getAmbientMode();
		float[] sample = new float[l1.sampleSize() + l2.sampleSize()];
		l1Prov.fetchSample(sample, 0);
		l2Prov.fetchSample(sample, 1);
		LCD.drawString(Float.toString(sample[0]*100), 0, 1);
		LCD.drawString(Float.toString(sample[1]*100), 0, 2);
		return (((int) (sample[0]*100 + sample[1]*100))/2);
	}
}
