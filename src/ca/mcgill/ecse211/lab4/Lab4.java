// Lab2.java
package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Class containing the main method, uses all the class interfaces to make the cart drive around a
 * set of waypoints.
 * 
 * @author angelortiz
 *
 */
public class Lab4 {

  // Motor objects
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();

  // Sensor objects
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the
                                                                               // instance
  private static final SampleProvider usDistance = usSensor.getMode("Distance");

  private static final Port lightPort = LocalEV3.get().getPort("S2");
  private static final EV3ColorSensor lightSensor = new EV3ColorSensor(lightPort); // usSensor is
                                                                                   // the instance
  private static final SampleProvider lightSensorProvider = lightSensor.getMode("Red");
  private static final MeanFilter lightFilter = new MeanFilter(lightSensorProvider, 4);

  // Parameters
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 15.2;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;
    boolean fallingEdge;

    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("  UP - Falling  ", 0, 0);
      lcd.drawString("      edge.     ", 0, 1);
      lcd.drawString(" DOWN - Rising  ", 0, 2);
      lcd.drawString("      edge.     ", 0, 3);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN);

    if (buttonChoice == Button.ID_UP) {
      fallingEdge = true;
    } else {
      fallingEdge = false;
    }

    // Odometer and navigation related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Display odometryDisplay = new Display(lcd);
    UltrasonicPoller usPoller = new UltrasonicPoller(usDistance);
    Navigation navigation =
        new Navigation(leftMotor, rightMotor, usPoller, WHEEL_RAD, TRACK, false);
    UltrasonicLocalizer uLocalizer =
        new UltrasonicLocalizer(leftMotor, rightMotor, usPoller, odometer, WHEEL_RAD, TRACK);
    LightLocalizer lLocalizer =
        new LightLocalizer(leftMotor, rightMotor, odometer, lightFilter, WHEEL_RAD, TRACK);

    // Start odometer and display threads
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
    Thread usPollerThread = new Thread(usPoller);
    usPollerThread.start();
    Thread navigationThread = new Thread(navigation);
    navigationThread.start();

    // Wait one second to ensure the ultrasonic sensor is on
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
    }

    // Perform falling edge or rising edge localization
    if (fallingEdge) {
      uLocalizer.fallingEdge();
    } else {
      uLocalizer.risingEdge();
    }

    // Turn to the absolute angle of 0 degrees
    navigation.turnTo(0);

    // Perform light localization
    lLocalizer.localize();

    // Go to the point [0, 0]
    navigation.travelTo(0, 0);

    // Wait until point [0, 0] is reached
    while (true) {
      try {
        Thread.sleep(70);
      } catch (InterruptedException e) {
      }
      if (!navigation.isNavigating())
        break;
    }

    // Turn to the absolute angle of 0 degrees again
    navigation.turnTo(0);

    System.exit(0);
  }
}
