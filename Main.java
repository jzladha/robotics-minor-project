import lejos.robotics.navigation.DifferentialPilot;
public class Main {
	private static float kp = 450;
	private static double kdiff = 500;
	private static float offset = (float) 0.4;
	private static float tp =  (float) (0.2 * Sensors.motorB.getMaxSpeed());
	private static DifferentialPilot pilot = new DifferentialPilot(5.5, 5.5, 12, Sensors.motorB, Sensors.motorD, false);
	private static float previousError = 0;
	private static float kd = (float) 1400;
	private static float distOffset = (float) 0.12;
	
	/**
	 * Default Constructor
	 */
	public Main() {
		// empty Constructor
	}
	
	/**
	 * Methods
	 */
	
	/**
	 * This is a Main method
	 * @author Vijay, Tamerlan, Marwan, Jazir
	 */
	public static void main(String[] args) {
		boolean isStopped = false;
		Sensors.motorB.setSpeed(tp);
		Sensors.motorD.setSpeed(tp);
		while (true){
			if(!Sensors.isRed()) {
				if(isStopped) {
					isStopped = false;
				}
					if(Sensors.getDistance() < 0.1) {
						pilot.quickStop();
						getPassObstacle();
					}
	        	
					float lightVal = Sensors.getColour();
					float error = lightVal - offset;
					previousError = error;
					double p =  getError(error);
					double d = getDError(error);
					Sensors.moveForward(tp, p, d);
			}
			else {
				pilot.quickStop();
				isStopped = true;
			}
		}
	}
	/**
	 * takes in previous error and returns new proportional error
	 * @author Marwan, Jazir and Tamerlan
	 */
	public static double getError(float error){	
    	double turn = (kp * error);
    	previousError = error;
    	return  turn;
	}
	/**
	 * returns derivative error
	 * @author Marwan, Jazir and Tamerlan
	 */
	public static double getDError(float error){
		float errorDelta = error - previousError;
		return kdiff * (errorDelta);
	}
	/**
	 * returns the proportional error for the object follower
	 * @author vijay
	 */
	private static float getDistError() {
		float distance = Sensors.getDistance();
		if(distance > 0.2) {
			float error = (float) (0.15 - distOffset);
			return (kd*error);
		}
		else if(distance == Float.POSITIVE_INFINITY) {
			float error  = (float) -0.08;
			return (kd*error);
		}
		else {
			float error = distance - distOffset;
			return (kd*error);
		}
	}
	/**
	 * moves across obstacle
	 * @author vijay
	 */
	private static void getPassObstacle() {
		Sensors.rotateRightAngle();
		Sensors.rotateHead(-90);
		while(Sensors.getColour() > offset) {
			float distErr = getDistError();
			Sensors.moveForwardacrossObject(tp, distErr);
		}
		pilot.quickStop();
		Sensors.rotateHead(90);
		pilot.travel(4.5);
		while(Sensors.getColour()> 0.1) {
			Sensors.rotate(10);
		}
		while(Sensors.getColour() < 0.4) {
			Sensors.rotate(10);
		}
	}
}
