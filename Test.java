import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;

public class Test {

    static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S4);
    static SampleProvider usDistance = usSensor.getDistanceMode();
    static float[] usSample = new float[usDistance.sampleSize()];

    static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
    static SampleProvider gyroAngle = gyroSensor.getAngleMode();
    static float[] angleSample = new float[gyroAngle.sampleSize()];
    
    static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
    static SampleProvider colorStrength = colorSensor.getAmbientMode();
    static float[] colorSample = new float[colorStrength.sampleSize()];
    
    static EV3LargeRegulatedMotor whipMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    static EV3MediumRegulatedMotor left = new EV3MediumRegulatedMotor(MotorPort.B);
    static EV3LargeRegulatedMotor right = new EV3LargeRegulatedMotor(MotorPort.D);

    static Wheel wheel1 = WheeledChassis.modelWheel(left, -5.6).offset(-13);
    static Wheel wheel2 = WheeledChassis.modelWheel(right, 5.6).offset(13);
    static MovePilot pilot = new MovePilot(new WheeledChassis(new Wheel[]{wheel1, wheel2}, WheeledChassis.TYPE_DIFFERENTIAL));

    public static void main(String[] args) {
        LCD.clear();
        LCD.drawString("ENTER Start", 0, 0);
        Button.ENTER.waitForPress();

        whipMotor.setSpeed(whipMotor.getMaxSpeed());
        whipMotor.setAcceleration(6000);
        pilot.setLinearSpeed(120);
        pilot.setAngularSpeed(140);

        gyroSensor.reset();
        Delay.msDelay(500);
        LCD.clear();

        float lastDistance = getDistance();
        int stuckCounter = 0;
        int suchCounter = 0;

        while (true) {
            if (Button.ESCAPE.isDown()) {
                cleanup();
                System.exit(0);
            }

            float currentDistance = getDistance();

            LCD.clear();
            LCD.drawString("Distanz: " + String.format("%.2f", currentDistance), 0, 0);

            // Gegnererkennung: Abstand plötzlich viel kleiner
            if (lastDistance - currentDistance > 0.15 && currentDistance < 0.4) {
                LCD.drawString("Gegner erkannt!", 0, 1);
                pilot.travel(-10);
                Delay.msDelay(50);
                peitscheSchlag(whipMotor);
                suchCounter = 0;
                Delay.msDelay(500);
            } else {
                // Zufällige Bewegung zur Suche
                suchCounter++;
                float angleBefore = getGyroAngle();
                int randomAngle = (int)(Math.random() * 100 - 50);
                pilot.rotate(randomAngle);

                // Prüfen ob Rotation erfolgreich war (Gyro)
                float angleAfter = getGyroAngle();
                float rotated = Math.abs(angleAfter - angleBefore);
                if (rotated < 5) {
                    LCD.drawString("Seite blockiert!", 0, 2);
                    pilot.travel(-10);
                    pilot.rotate((Math.random() > 0.5 ? 90 : -90));
                    Delay.msDelay(200);
                }

                if (Math.random() <= 0.5 && currentDistance > 0.15) {
                    pilot.travel(15);
                }

                if (suchCounter >= 5) {
                    suchCounter = 0;

                    Thread peitschenThread1 = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            peitscheSchlag(whipMotor);
                        }
                    });

                    peitschenThread1.start();

                    pilot.rotate(70);    // Erst +70°
                    Delay.msDelay(100);

                    try {
                        peitschenThread1.join();  // Warten bis fertig
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Thread peitschenThread2 = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            peitscheSchlag(whipMotor);
                        }
                    });

                    peitschenThread2.start();

                    pilot.rotate(-140);  // Dann -140° ergibt -70° vom Start
                    Delay.msDelay(100);

                    try {
                        peitschenThread2.join();  // Warten bis fertig
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Thread peitschenThread3 = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            peitscheSchlag(whipMotor);
                        }
                    });

                    peitschenThread3.start();

                    pilot.rotate(70);    // Erst +70°
                    Delay.msDelay(100);

                    try {
                        peitschenThread3.join();  // Warten bis fertig
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    Thread peitschenThread4 = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            peitscheSchlag(whipMotor);
                        }
                    });

                    peitschenThread4.start();

                    pilot.rotate(-140);  // Dann -140° ergibt -70° vom Start
                    Delay.msDelay(100);

                    try {
                        peitschenThread4.join();  // Warten bis fertig
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                 
                if (wandHintenErkennung() < 0.1) {
                    LCD.drawString("Wand hinten!", 0, 4);
                    pilot.travel(30);
                    pilot.rotate((Math.random() > 0.5 ? 50 : -50));
                    Delay.msDelay(200);
                }
            }

            // Blockadeerkennung (z.B. vor Wand)
            if (Math.abs(currentDistance - lastDistance) < 0.01 && currentDistance < 0.1) {
                stuckCounter += 50;
                if (stuckCounter >= 4000) {
                    pilot.travel(-20);
                    pilot.rotate((Math.random() > 0.5 ? 90 : -90));
                    stuckCounter = 0;
                    suchCounter = 0;
                }
            } else {
                stuckCounter = 0;
            }

            lastDistance = currentDistance;
            Delay.msDelay(50);
        } 
    }

    public static float getDistance() {
        usDistance.fetchSample(usSample, 0);
        return usSample[0];
    }

    public static float getGyroAngle() {
        gyroAngle.fetchSample(angleSample, 0);
        return angleSample[0];
    }

    // peitscheSchlag Methode mit Async und Timeout:
    public static void peitscheSchlag(EV3LargeRegulatedMotor motor) {
        motor.rotate(-250, true);  // true = nicht blockieren
        waitForMotor(motor, 1500);

        motor.rotate(250, true);
        waitForMotor(motor, 1500);
    }

    private static void waitForMotor(EV3LargeRegulatedMotor motor, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (motor.isMoving()) {
            if (System.currentTimeMillis() - start > timeoutMs) {
                motor.stop();
                break;
            }
            Delay.msDelay(10);
        }
    }
    
    public static float wandHintenErkennung() {
    	colorStrength.fetchSample(colorSample, 0);
        return colorSample[0];
    }

    public static void cleanup() {
        whipMotor.close();
        left.close();
        right.close();
        usSensor.close();
        gyroSensor.close();
    }
}
