import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
public class Sensors {
	public static EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.B);
	public static EV3LargeRegulatedMotor motorD = new EV3LargeRegulatedMotor(MotorPort.D);
	public static EV3MediumRegulatedMotor motorA = new EV3MediumRegulatedMotor(MotorPort.A);
	public static EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor (SensorPort.S4);
	public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	public static EV3LargeRegulatedMotor[] list = {motorD};
	/**
	 * Default Constructor
	 */
	public Sensors() {
		//empty constructor
	}
	
	/**
	 * main functions
	 */
	
	/**
	 * this method returns the distance between robot and object its facing
	 * @author vijay
	 */
	public static float getDistance(){
    	SampleProvider sp1 = ultraSensor.getDistanceMode();
    	float[] sample = new float[sp1.sampleSize()];
    	sp1.fetchSample(sample, 0);
    	return sample[0];
	}
	/**
	 * this method returns the colour Id Value
	 * @author Marwan, Jazir and Tamerlan
	 */
	public static float getColour(){
    	SampleProvider colorProvider = colorSensor.getRedMode();
    	float[] sample = new float[colorProvider.sampleSize()];
    	colorProvider.fetchSample(sample, 0);
    	return sample[0];
	}
	/**
	 * checks for red curtain
	 * @author Tamerlan
	 */
	public static boolean isRed() {
		return (colorSensor.getColorID() == Color.RED);
	}
	/**
	 * moves the robot forward according to tp, p, d
	 * @author Marwan, Jazir and Tamerlan
	 */
	public static void moveForward(float tp, double p, double d) {
		float speedB = (float)(tp - (p + d));
		float speedD = (float)(tp +  p + d);
		if(speedB > speedD) {
		 motorB.setSpeed(speedB);
		 motorD.setSpeed(speedD);
		}
		else {
		 motorB.setSpeed(speedB);
		 motorD.setSpeed((float) (speedD+25));
		}
		motorB.synchronizeWith(list);
		motorB.startSynchronization();
		motorB.forward();
		motorD.forward();
		motorB.endSynchronization();
	}
	/**
	 * makes the robot rotate 90 degrees right
	 * @author vijay
	 */
	public static void rotateRightAngle() {
		motorB.setSpeed(100);
		motorD.setSpeed(100);
		motorB.synchronizeWith(list);
		motorB.startSynchronization();
		motorB.rotate(180);
		motorD.rotate(-180);
		motorB.endSynchronization();
		motorB.waitComplete();
	}
	/**
	 * rotates the head
	 * @author vijay
	 */
	public static void rotateHead(int angle) {
		motorA.rotate(angle);
	}
	/**
	 * moves the robot forward according to tp and distErr
	 * @author vijay
	 */
	public static void moveForwardacrossObject(float tp, float distErr) {
		motorB.setSpeed(tp - distErr);
		motorD.setSpeed(tp + distErr);
		motorB.startSynchronization();
		motorB.forward();
		motorD.forward();
		motorB.endSynchronization();
	}
	/**
	 * rotates the robot to given angle in degrees
	 */
	public static void rotate(int angle) {
		motorB.setSpeed(1000);
		motorD.setSpeed(1000);
		motorB.startSynchronization();
		motorB.rotate(angle);
		motorD.rotate(-angle);
		motorB.endSynchronization();
		motorB.waitComplete();
	}
}
