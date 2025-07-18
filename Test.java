import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Test {

    // Sensoren
    static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
    static SampleProvider usDistance = usSensor.getDistanceMode();
    static float[] usSample = new float[usDistance.sampleSize()];

    static EV3ColorSensor rgbSensor = new EV3ColorSensor(SensorPort.S1);
    static SampleProvider rgb = rgbSensor.getRGBMode();
    static float[] rgbSample = new float[rgb.sampleSize()];

    public static void main(String[] args) {
        int stuckCounter = 0;
        int suchCounter = 0;

        EV3LargeRegulatedMotor whipMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        whipMotor.setSpeed(whipMotor.getMaxSpeed());
        whipMotor.setAcceleration(6000);

        EV3MediumRegulatedMotor left = new EV3MediumRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor right = new EV3LargeRegulatedMotor(MotorPort.D);

        Wheel wheel1 = WheeledChassis.modelWheel(left, -5.6).offset(-13);
        Wheel wheel2 = WheeledChassis.modelWheel(right, 5.6).offset(13);
        MovePilot pilot = new MovePilot(new WheeledChassis(new Wheel[]{wheel1, wheel2}, WheeledChassis.TYPE_DIFFERENTIAL));
        pilot.setLinearSpeed(90);
        pilot.setAngularSpeed(140);

        System.out.println("Drücke ENTER zum Starten");
        Button.ENTER.waitForPress();

        float lastDistance = getUltrasonicDistance();
        
        while (true) {

            if (Button.ESCAPE.isDown()) {
                System.out.println("Beende Programm.");
                cleanup(whipMotor, left, right);
                System.exit(0);
                break;
            }
            
            if (Button.DOWN.isDown()) {
            	peitscheSchlag(whipMotor);
            }

            // Rot erkennen
            if (erkenneRotHSV()) {
                pilot.travel(-10);
                peitscheSchlag(whipMotor);
                System.out.println("ROT!");
                pilot.travel(-20);
                suchCounter = 0; // Reset Suche
            }

            float currentDistance = getUltrasonicDistance();

            // Gegner erkannt?
            if (lastDistance - currentDistance > 0.1 && currentDistance < 0.2) {
                System.out.println("Gegner erkannt bei " + currentDistance + " m");
                pilot.travel(15);
                Delay.msDelay(50);

                if (erkenneRotHSV()) {
                    pilot.travel(-10);
                    peitscheSchlag(whipMotor);
                    System.out.println("ROT!");
                }
                suchCounter = 0; // Reset Suche
            } else {
                // Gegner nicht gefunden — Suchbewegung
                suchCounter++;
                System.out.println("Suche... Versuch " + suchCounter + ", Abstand: " + currentDistance);

                pilot.rotate((int) (Math.random() * 140 - 70));

                if (erkenneRotHSV()) {
                    System.out.println("ROT!");
                    pilot.travel(-10);
                    peitscheSchlag(whipMotor);
                    suchCounter = 0;
                }

                if ((Math.random() <= 0.4) && currentDistance > 0.15) {
                    pilot.travel(15);
                }

                // Nach 7 erfolglosen Suchen
                if (suchCounter >= 7) {
                    System.out.println("Nichts gefunden DREH");
                    pilot.travel(-20);
                    suchCounter = 0;
                    pilot.setAngularSpeed(240);
                    pilot.rotate(1080);
                    pilot.setAngularSpeed(120);
                }
            }

            // Wand-Block-Check
            if (Math.abs(currentDistance - lastDistance) < 0.01 && currentDistance < 0.1) {
                stuckCounter += 50;
                if (stuckCounter >= 4000) {
                    System.out.println("Blockiert Wand ");
                    pilot.travel(-20);
                    pilot.rotate((Math.random() > 0.5 ? 90 : -90));
                    stuckCounter = 0;
                    suchCounter = 0; // Reset auch hier
                }
            } else {
                stuckCounter = 0;
                
            }

            lastDistance = currentDistance;
            Delay.msDelay(50);
        }
    }


    // ==== Sensor- & Aktionsmethoden ====

    public static float getUltrasonicDistance() {
        usDistance.fetchSample(usSample, 0);
        return usSample[0];
    }

    /**
     * Liest RGB vom EV3-Farbsensor, wandelt in HSV um und prüft auf roten Farbton.
     * @return true, wenn ein heller Rottone erkannt wird.
     */
    
    public static boolean erkenneRotHSV() {
        // 3 Messungen mitteln
    	rgb.fetchSample(rgbSample, 0);
        float r = rgbSample[0];
        float g = rgbSample[1];
        float b = rgbSample[2];

        // HSV-Berechnung
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float h;
        if (delta == 0) {
            h = 0;
        } else if (max == r) {
            h = 60 * (((g - b) / delta) % 6);
        } else if (max == g) {
            h = 60 * (((b - r) / delta) + 2);
        } else {
            h = 60 * (((r - g) / delta) + 4);
        }
        if (h < 0) h += 360;

        float s = (max == 0) ? 0 : (delta / max);
        float v = max;

        // Roterkennung
        boolean istRot = ( (h < 40 || h > 240) && s > 0.2 && v >= 0.0039 );

        // Anzeige auf dem EV3-Screen
        LCD.clear();
        LCD.drawString(String.format(""), 0, 0);
        LCD.drawString(String.format("H: %.1f", h), 0, 1);
        LCD.drawString(String.format("S: %.2f", s), 0, 2);
        LCD.drawString(String.format("V: %.5f", v), 0, 3);
        LCD.drawString("ROT: " + (istRot ? "JA" : "NEIN"), 0, 5);
        Delay.msDelay(100);
        return istRot;
    }


  

    public static void peitscheSchlag(EV3LargeRegulatedMotor motor) {
    	
        motor.backward();
        Delay.msDelay(600);
        motor.stop();
        motor.rotate(290);
        motor.stop();

    }

    public static void peitscheEinfahren(EV3LargeRegulatedMotor motor) {
        motor.rotate(-280);
        motor.stop();
    }

    public static void cleanup(EV3LargeRegulatedMotor whipMotor, EV3MediumRegulatedMotor left, EV3LargeRegulatedMotor right) {
        whipMotor.close();
        left.close();
        right.close();
        usSensor.close();
        rgbSensor.close();

    }
}