package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection.Direction;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * The UltrasonicLocalizer provides an interface to correct the odometer's angle reading by
 * performing wall detection with the ultrasonic sensor. This routine can be performed regardless of
 * the initial angle in which the robot is placed into the grid.
 * 
 * @author angelortiz
 *
 */
public class UltrasonicLocalizer {

  // Constants
  private static final int ROTATE_SPEED = 100;
  private static final int THRESHOLD = 40;
  private static final int LOCALIZATION_PERIOD = 25;

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private UltrasonicPoller usPoller;
  private Odometer odometer;
  private int prevDistance;
  private int currDistance;
  private boolean measurementTaken;
  private double alpha;
  private double beta;

  public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      UltrasonicPoller usPoller, Odometer odometer, double wheelRadius, double track) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.usPoller = usPoller;
    this.odometer = odometer;
    this.alpha = 0;
    this.beta = 0;
  }

  /**
   * This method rotates the robot in the clockwise direction until a falling edge is detected by
   * the ultrasonic sensor, then the robot rotates in the counter-clockwise direction until a second
   * falling edge is detected. The two angles are recorded and used to correct the odometer's angle
   * reading.
   */
  public void fallingEdge() {
    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Rising edge
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.distance;
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }
      if (prevDistance >= currDistance && prevDistance > THRESHOLD) { // Distance decreasing
        if (!measurementTaken && currDistance < THRESHOLD) {
          if (firstSearch) {
            beta = odometer.getXYT()[2];
            leftMotor.backward();
            rightMotor.forward();
            firstSearch = false;
            measurementTaken = true;
          } else {
            alpha = odometer.getXYT()[2];
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
        }
      } else if (prevDistance < currDistance) { // Distance increasing
        measurementTaken = false;
      }
      prevDistance = currDistance;
      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    correctAngle();
  }

  /**
   * This method rotates the robot in the clockwise direction until a rising edge is detected by the
   * ultrasonic sensor, then the robot rotates in the counter-clockwise direction until a second
   * rising edge is detected. The two angles are recorded and used to correct the odometer's angle
   * reading.
   */
  public void risingEdge() {
    long updateStart, updateEnd;
    prevDistance = -1;
    boolean firstSearch = true;

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    // Rising edge
    while (true) {
      updateStart = System.currentTimeMillis();
      currDistance = usPoller.distance;
      if (prevDistance < 0) {
        prevDistance = currDistance;
        continue;
      }
      if (prevDistance <= currDistance && prevDistance < THRESHOLD) { // Distance increasing
        if (!measurementTaken && currDistance > THRESHOLD) {
          if (firstSearch) {
            alpha = odometer.getXYT()[2];
            leftMotor.backward();
            rightMotor.forward();
            firstSearch = false;
            measurementTaken = true;
          } else {
            beta = odometer.getXYT()[2];
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            break;
          }
        }
      } else if (prevDistance > currDistance) { // Distance decreasing
        measurementTaken = false;
      }
      prevDistance = currDistance;
      // This ensures that the navigator only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < LOCALIZATION_PERIOD) {
        try {
          Thread.sleep(LOCALIZATION_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
    correctAngle();
  }

  /**
   * Uses angle measurements from rising or falling edge methods to correct the odometer's theta
   * reading.
   */
  private void correctAngle() {
    double[] position = odometer.getXYT();
    double correctedTheta;
    if (alpha < beta) {
      correctedTheta = position[2] + 45 - ((alpha + beta) / 2);
    } else {
      correctedTheta = position[2] + 225 - ((alpha + beta) / 2);
    }
    odometer.setTheta(correctedTheta);
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

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
