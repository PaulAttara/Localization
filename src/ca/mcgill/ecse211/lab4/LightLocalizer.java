package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection.Direction;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * The LightLocalizer class provides the functionality for correcting the X and Y values of the
 * odometer using a line detection algorithm with the light sensor. For this subroutine to work the
 * robot must be positioned at absolute angle zero.
 * 
 * @author angelortiz
 *
 */
public class LightLocalizer {

  // Constants
  private static final int ROTATE_SPEED = 100;
  private static final double SENSOR_OFFSET = 12.3;
  private static final long LINE_DETECTION_PERIOD = 10;
  private static final double LINE_COLOR_VALUE = 0.14; // Minimum value required to treat a sensor
                                                       // reading as a line

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Odometer odometer;
  private MeanFilter lightSensor;
  private double wheelRadius;
  private double track;

  public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Odometer odometer, MeanFilter lightSensor, double wheelRadius, double track) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;
    this.lightSensor = lightSensor;
    this.wheelRadius = wheelRadius;
    this.track = track;
  }

  /**
   * Corrects the odometer's X and Y values by moving the robot back and forth in each direction
   * until a line is detected.
   */
  public void localize() {
    float[] sample = new float[lightSensor.sampleSize()];
    boolean inLine = false; // Flags whether the sensor is currently seeing a line
    boolean xDone = false;
    boolean yDone = false;

    long correctionStart, correctionEnd;

    // Save the starting position
    double[] startingPosition = odometer.getXYT();

    // Start moving forward until a line is detected
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    // Run until both x and y are corrected in the odometer
    while (!xDone || !yDone) {
      correctionStart = System.currentTimeMillis();

      lightSensor.fetchSample(sample, 0);
      boolean lineDetected = sample[0] < LINE_COLOR_VALUE;

      // Trigger correction (When do I have information to correct?)
      if (lineDetected) {
        if (!inLine) {
          inLine = true;
          Sound.beep(); // Indicate that a line is detected
          if (!yDone) {
            double[] currPosition = odometer.getXYT();

            // Correct the y value of the odometer
            odometer.setXYT(currPosition[0], SENSOR_OFFSET, currPosition[2]);
            yDone = true;

            // Go back to starting position
            leftMotor.rotate(convertDistance(wheelRadius, startingPosition[1] - currPosition[1]),
                true);
            rightMotor.rotate(convertDistance(wheelRadius, startingPosition[1] - currPosition[1]),
                false);

            // Rotate 90 degrees to correct x
            leftMotor.setSpeed(ROTATE_SPEED);
            rightMotor.setSpeed(ROTATE_SPEED);
            leftMotor.rotate(convertAngle(wheelRadius, track, 90), true);
            rightMotor.rotate(-convertAngle(wheelRadius, track, 90), false);

            // Reset the starting position value using the corrected odometer y value
            startingPosition = odometer.getXYT();

            // Move forward again
            leftMotor.forward();
            rightMotor.forward();
          } else if (!xDone) {
            double[] currPosition = odometer.getXYT();

            // Correct the x value of the odometer
            odometer.setXYT(SENSOR_OFFSET, currPosition[1], currPosition[2]);
            xDone = true;

            // Move back to the starting position
            leftMotor.rotate(convertDistance(wheelRadius, startingPosition[0] - currPosition[0]),
                true);
            rightMotor.rotate(convertDistance(wheelRadius, startingPosition[0] - currPosition[0]),
                false);

            // Rotate -90 degrees to return to original position
            leftMotor.setSpeed(ROTATE_SPEED);
            rightMotor.setSpeed(ROTATE_SPEED);
            leftMotor.rotate(convertAngle(wheelRadius, track, -90), true);
            rightMotor.rotate(-convertAngle(wheelRadius, track, -90), false);
          }

        }
      } else
        inLine = false;

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < LINE_DETECTION_PERIOD) {
        try {
          Thread.sleep(LINE_DETECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
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

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
