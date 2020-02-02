package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LightLocalizer implements TimerListener {

  private static final Port lightPort = LocalEV3.get().getPort("S2");
  private static final SensorModes lightMode = new EV3ColorSensor(lightPort);
  private static final SampleProvider lightSampleProvider = lightMode.getMode("Red");
  private float[] sample = new float[lightSampleProvider.sampleSize()];

  private final double TILE_COLOR = 0.35;
  // these readings comes from sensor test
  // blue board: 0.34 line: 0.17-0.2
  // yellow board: 0.39 line: 0.21
  // set the tile color to 0.35, so the light sensor can work on both boards

  private static Odometer odo = null;
  private static int count = 0;



  // in degreenitialize them to invalid angle data
  private static double startOfThetaY = -1;
  private static double endOfThetaY = -1;
  private static double startOfThetaX = -1;
  private static double endOfThetaX = -1;


  public LightLocalizer() throws OdometerExceptions {
    odo = Odometer.getOdometer();

    Lab4.leftMotor.setAcceleration(Lab4.ACCELERATION);
    Lab4.rightMotor.setAcceleration(Lab4.ACCELERATION);
  }

  /**
   * this method moves the robot closer to (0,0) and then turn to 0
   * after that the robot will turn 360 anti-clockwise and do the light localization
   * when the light localization is done, the robot will move to (0,0) and turn to 0
   * @throws OdometerExceptions
   */
  public void localizer() throws OdometerExceptions {

    Lab4.leftMotor.setSpeed(Lab4.TURNING_SPEED);
    Lab4.rightMotor.setSpeed(Lab4.TURNING_SPEED);

    // move to (0,0)
    Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), true);
    Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), false);

    Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), true);
    Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 8), false);

    Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), true);
    Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), false);

    Timer timer = new Timer(50, new LightLocalizer());
    timer.start();

    Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360), true);
    Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360), false);

    double currentTheta = odo.getXYT()[2];

    // calculate x correction
    double thetaY = startOfThetaY - endOfThetaY;
    double xCorrection = -Lab4.LIGHT_SENSOR_TO_ROBOT_CENTER * Math.cos(Math.toRadians(thetaY / 2));

    // calculate y correction
    double thetaX = startOfThetaX - endOfThetaX;
    double yCorrection = -Lab4.LIGHT_SENSOR_TO_ROBOT_CENTER * Math.cos(Math.toRadians(thetaX / 2));

    // calculate theta correction
    double deltaTheta = 90 - (startOfThetaY - 180) + thetaY / 2;

    // perform correction
    odo.setXYT(xCorrection, yCorrection, deltaTheta);

    travelTo(0, 0);
    turnTo(0);
  }


  /**
   * a method from TimerListner interface it fetches reading from the light sensor when time out and
   * record the angle readings from the odometer if the light sensor detects lines
   */
  @Override
  public void timedOut() {
    lightSampleProvider.fetchSample(sample, 0);
    double currentAngle = odo.getXYT()[2];

    if (TILE_COLOR - sample[0] > 0.1) {
      // 0.1 is the different between tile color and line, it comes from the sensor test metioned
      // above
      if (count <= 4) {
        Sound.beep();
      }
      count++;
      switch (count) {
        case 1: {
          startOfThetaY = currentAngle;
          break;
        }
        case 2: {
          startOfThetaX = currentAngle;
          break;
        }
        case 3: {
          endOfThetaY = currentAngle;
          break;
        }
        case 4: {
          endOfThetaX = currentAngle;
          break;
        }

      }

    }
  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * this method calculate the distance that the robot needed to travel when finishing the turn
   * 
   * @param radius
   * @param width
   * @param angle
   * @return the distance that the robot needed to travel when finishing the turn
   * 
   */
  private static int convertAngle(double radius, double width, double angle) {
    // angle >= 0
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * this method cause the robot to travel to the absolute field location (x,y)
   * 
   * @param x
   * @param y
   * 
   */
  void travelTo(double x, double y) {


    double pos[] = odo.getXYT();
    double currentX = pos[0];
    double currentY = pos[1];
    double theta = pos[2];

    double distance = Math.sqrt(Math.pow((currentX - x * Lab4.TILE_SIZE), 2)
        + Math.pow((currentY - y * Lab4.TILE_SIZE), 2));

    turnTo(angleCalculation(currentX, currentY, x * Lab4.TILE_SIZE, y * Lab4.TILE_SIZE));

    Lab4.leftMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.rightMotor.setSpeed(Lab4.FORWARD_SPEED);
    Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), true);
    Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), false);


  }

  /**
   * 
   * @param currentX
   * @param currentY
   * @param x
   * @param y
   * @return angle needs to turn this method calculate the angle that the robot needs to turn when
   *         traveling to the destination
   */
  private double angleCalculation(double currentX, double currentY, double x, double y) {

    // theta is in degree
    double dy = y - currentY;
    double dx = x - currentX;
    double dt = 0;
    if (dy >= 0) {
      dt = Math.atan(dx / dy);
    } else if (dy <= 0 && dx >= 0) {
      dt = Math.atan(dx / dy) + Math.PI;
    } else {
      dt = Math.atan(dx / dy) - Math.PI;
    }

    return dt * 180 / Math.PI;
  }

  /**
   * this method causes the robot to turn to the absolute heading theta the method turn a minimal
   * angel to its target
   * 
   * @param theta
   */
  void turnTo(double theta) {

    double currentTheta = odo.getXYT()[2];
    double turningAngle = theta - currentTheta;

    if (Math.abs(turningAngle) <= 180) {
      if (turningAngle <= 0) {
        // turn left with -turningAngle
        Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -turningAngle), true);
        Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -turningAngle), false);
      } else {
        // turn right with turningAngle
        Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), true);
        Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), false);
      }
    } else if (Math.abs(turningAngle) > 180) {
      if (turningAngle < 0) {
        // turn right with 360+turning angle
        Lab4.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360 + turningAngle), true);
        Lab4.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360 + turningAngle),
            false);
      } else {
        // turn left with 360 - turningAngle
        Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360 - turningAngle), true);
        Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360 - turningAngle), false);
      }
    }

  }

}
