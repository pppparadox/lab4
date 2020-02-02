package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab4 {

  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();

  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 9.8;
  public static final double LIGHT_SENSOR_TO_ROBOT_CENTER = 14;

  public static final int TURNING_SPEED = 50;
  public static final int FORWARD_SPEED = 150;
  public static final int ACCELERATION = 300;
  public static final double TILE_SIZE = 30.48;



  public static void main(String[] args) throws OdometerExceptions, InterruptedException {

    int buttonSelection;


    do {
      // display menu
      lcd.clear();

      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString(" Fall- | Ris-   ", 0, 1);
      lcd.drawString(" ing   | ing    ", 0, 2);
      lcd.drawString(" edge  | edge   ", 0, 3);
      lcd.drawString("       |        ", 0, 4);
      buttonSelection = Button.waitForAnyPress();
    } while (buttonSelection != Button.ID_LEFT && buttonSelection != Button.ID_RIGHT);

    // initialize and start odometer thread
    Odometer odo = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    Thread odoThread = new Thread(odo);
    odoThread.start();

    // initialize and start lcd display thread
    Display display = new Display(lcd);
    Thread displayThread = new Thread(display);
    displayThread.start();

    // initialize UltrasonicLocalizer
    final UltrasonicLocalizer ul = new UltrasonicLocalizer();

    // initialize LightLocalizer
    final LightLocalizer ll = new LightLocalizer();


    if (buttonSelection == Button.ID_LEFT) {
      // start falling edge if the left button is pressed

      Thread ulThread = new Thread() {
        public void run() {
          try {
            ul.fallingEdge();
          } catch (OdometerExceptions e) {
            e.printStackTrace();
          }
        }
      };
      ulThread.start();

      // wait for the error measurement of the ultrasonic localizer
      while (Button.waitForAnyPress() != Button.ID_RIGHT);

      // start the light localizer
      if (!ulThread.isAlive()) {
        Thread llr = (new Thread() {
          public void run() {
            try {
              ll.localizer();
            } catch (OdometerExceptions e) {
              e.printStackTrace();
            }
          }
        });
        Thread.sleep(1000);
        // pause for 1s
        llr.start();

      }
    } else if (buttonSelection == Button.ID_RIGHT) {
      // start rising edge if the right button is pressed
      Thread ulThread1 = new Thread() {
        public void run() {
          try {
            ul.risingEdge();
          } catch (OdometerExceptions e) {
            e.printStackTrace();
          }
        }
      };
      ulThread1.start();
      // wait for the error measurement of the ultrasonic localizer
      while (Button.waitForAnyPress() != Button.ID_RIGHT);

      // start the light localizer
      if (!ulThread1.isAlive()) {
        Thread llr = (new Thread() {
          public void run() {
            try {
              ll.localizer();
            } catch (OdometerExceptions e) {
              e.printStackTrace();
            }
          }
        });
        Thread.sleep(1000);
        // pause for 1s
        llr.start();
      }
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);

  }
}
