import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;


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

    // Flags, die von den Sensor-Threads gesetzt werden
    static volatile boolean gegnerErkannt = false;
    static volatile boolean wandHinten = false;

    public static void main(String[] args) {
        LCD.clear();
        LCD.drawString("ENTER Start", 0, 0);
        Button.ENTER.waitForPress();

        whipMotor.setSpeed(whipMotor.getMaxSpeed());
        whipMotor.setAcceleration(6000);
        pilot.setLinearSpeed(140);
        pilot.setAngularSpeed(160);

        gyroSensor.reset();
        Delay.msDelay(500);
        LCD.clear();

        // Start paralleler Sensorüberwachungs-Threads
        startGegnerErkennungThread();
        startFarbErkennungHintenThread();


        int suchCounter = 0;

        while (true) {
            if (Button.ESCAPE.isDown()) {
                cleanup();
                System.exit(0);
            }

            LCD.clear();

            if (gegnerErkannt) {
                LCD.drawString("Gegner erkannt", 0, 1);
                pilot.travel(-10);
                Delay.msDelay(50);
                peitscheSchlag(whipMotor);
                suchCounter = 0;
                Delay.msDelay(500);
                gegnerErkannt = false;  // zurücksetzen
            } else {
                suchCounter++;
                float angleBefore = getGyroAngle();
                int randomAngle = (int)(Math.random() * 100 - 50);
                pilot.rotate(randomAngle);

                float angleAfter = getGyroAngle();
                float rotated = Math.abs(angleAfter - angleBefore);
                if (rotated < 5) {
                    LCD.drawString("Seite blockiert", 0, 2);
                    pilot.travel(-10);
                    pilot.rotate((Math.random() > 0.5 ? 90 : -90));
                    Delay.msDelay(200);
                }

                if (Math.random() <= 0.5) {
                    pilot.travel(20);
                }

                if (suchCounter >= 5) {
                    suchCounter = 0;
                    peitschenSchwenk();
                }

                if (wandHinten) {
                    LCD.drawString("Wand hinten", 0, 4);
                    pilot.travel(30);
                    pilot.rotate((Math.random() > 0.5 ? 50 : -50));
                    Delay.msDelay(200);
                    wandHinten = false;  // zurücksetzen
                }
            }

            Delay.msDelay(50);
        }
    }

    public static void startGegnerErkennungThread() {
        Thread thread = new Thread(new Runnable() {
            public void run() {
                float lastDist = getDistance();
                while (true) {
                    float current = getDistance();
                    if ((lastDist - current > 0.15) && current < 0.4) {
                        gegnerErkannt = true;
                    }
                    lastDist = current;
                    Delay.msDelay(100);
                }
            }
        });
        thread.setDaemon(true);
        thread.start();
    }

    public static void startFarbErkennungHintenThread() {
        Thread thread = new Thread(new Runnable() {
            public void run() {
                while (true) {
                    float brightness = wandHintenErkennung();
                    if (brightness < 0.05f) {
                        wandHinten = true;
                    }
                    Delay.msDelay(100);
                }
            }
        });
        thread.setDaemon(true);
        thread.start();
    }

    public static float getDistance() {
        usDistance.fetchSample(usSample, 0);
        return usSample[0];
    }

    public static float getGyroAngle() {
        gyroAngle.fetchSample(angleSample, 0);
        return angleSample[0];
    }

    public static void peitscheSchlag(EV3LargeRegulatedMotor motor) {
        motor.rotate(-250, true);
        waitForMotor(motor, 1500);

        motor.rotate(250, true);
        waitForMotor(motor, 1500);
    }

    private static void peitschenSchwenk() {
        Thread t1 = new Thread(new Runnable() {
            public void run() {
                peitscheSchlag(whipMotor);
            }
        });
        t1.start();
        pilot.rotate(70);
        try {
            t1.join();
        } catch (InterruptedException e) {
        }

        Thread t2 = new Thread(new Runnable() {
            public void run() {
                peitscheSchlag(whipMotor);
            }
        });
        t2.start();
        pilot.rotate(-140);
        try {
            t2.join();
        } catch (InterruptedException e) {
        }
        
        Thread t3 = new Thread(new Runnable() {
            public void run() {
                peitscheSchlag(whipMotor);
            }
        });
        t3.start();
        pilot.rotate(70);
        try {
            t3.join();
        } catch (InterruptedException e) {
        }

        Thread t4 = new Thread(new Runnable() {
            public void run() {
                peitscheSchlag(whipMotor);
            }
        });
        t4.start();
        pilot.rotate(-140);
        try {
            t4.join();
        } catch (InterruptedException e) {
        }
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
