package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BumperRotation {

    public static double Rotate(double currentOrientation, String bumperPressed, Telemetry telemetry)
    {
        List<Double> allPossibleValues = Arrays.asList(0d, 45d, 90d, 135d, 180d, -45d, -90d, -135d);

        if(bumperPressed == "right")
        {
            if(currentOrientation < 45d && currentOrientation > 0)
            {
                telemetry.addData("return wanted orientation", 0);

                return 0d;
            }
            else if(currentOrientation < 90d && currentOrientation > 45)
            {
                telemetry.addData("return wanted orientation", 45);
                return 45d;
            }
            else if(currentOrientation < 135d && currentOrientation > 90)
            {
                telemetry.addData("return wanted orientation", 90);

                return 90d;
            }
            else if(currentOrientation < 179d && currentOrientation > 135)
            {
                telemetry.addData("return wanted orientation", 135);
                return 135d;

            }
            else if (currentOrientation < -135d && currentOrientation > -179)
            {
                telemetry.addData("return wanted orientation", -180);
                return -179d;
            }
            else if (currentOrientation < -90d && currentOrientation > -135)
            {
                telemetry.addData("return wanted orientation", -135);
                return -135d;
            }
            else if (currentOrientation < -45 && currentOrientation > -90)
            {
                telemetry.addData("return wanted orientation", -90);
                return -90d;
            }
            else if (currentOrientation < 0 && currentOrientation > -45)
            {
                telemetry.addData("return wanted orientation", -45);
                return -45d;
            }
            else return -179;
        }
        if(bumperPressed == "left")
        {
            if(currentOrientation > 0 && currentOrientation < 45)
            {
                return 45d;
            }
            if(currentOrientation > 45 && currentOrientation < 90)
            {
                return 90d;
            }
            if(currentOrientation > 90 && currentOrientation < 135)
            {
                return 135;
            }
            if(currentOrientation > 135 && currentOrientation < 179)
            {
                return 180;
            }
            if(currentOrientation > -179 && currentOrientation < -135)
            {
                return -135;
            }
            if(currentOrientation > -135 && currentOrientation < -90)
            {
                return -90;
            }
            if(currentOrientation > -90 && currentOrientation < -45)
            {
                return  -45;
            }
            if(currentOrientation > -45 && currentOrientation < 0)
            {
                return 0;
            }
            return -179;
        }
        else return 0;







        // old stuff
       /* List<Double> allPossibleValues = Arrays.asList(0d, 45d, 90d, 135d, 180d, 225d, 270d, 315d);

        List<Double> needThisIG= Arrays.asList();

        List<Double> valuesCalculated = new ArrayList<>(needThisIG);
        if(currentOrientation < 0) currentOrientation = currentOrientation * -1 + 180;
        if(bumperPressed == "right")
        {

                double targetedValue = currentOrientation - 45;
                if(targetedValue < 0) targetedValue += 360;
                double valueToAdd = Math.abs(Math.abs(allPossibleValues.get(0)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(1)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(2)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(3)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(4)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(5)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(6)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);
                valueToAdd = Math.abs(Math.abs(allPossibleValues.get(7)) - Math.abs(targetedValue));
                valuesCalculated.add(valueToAdd);



                telemetry.addData("All Values", valuesCalculated);
               double min = valuesCalculated.get(0);
                for (int i = 1; i < valuesCalculated.size(); i++) {
                    if (valuesCalculated.get(i) < min) {
                        min = valuesCalculated.get(i);

                    }
                }
                telemetry.addData("trun to" , min - currentOrientation);

                return (float) min - (float) currentOrientation;
        }
        if(bumperPressed == "left")
        {

            double targetedValue = currentOrientation + 45;
            if(targetedValue > 360) targetedValue = Math.abs(targetedValue - 360);

            double valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(0)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(1)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(2)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(3)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(4)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(5)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(6)));
            valuesCalculated.add(valueToAdd);
            valueToAdd = Math.abs(Math.abs(targetedValue) - Math.abs(allPossibleValues.get(7)));
            valuesCalculated.add(valueToAdd);



            telemetry.addData("All Values", valuesCalculated);
            double min = valuesCalculated.get(0);
            for (int i = 1; i < valuesCalculated.size(); i++) {
                if (valuesCalculated.get(i) < min) {
                    min = valuesCalculated.get(i);

                }
            }
            telemetry.addData("lowest Value", min);

            return (float) min + (float) currentOrientation;
        }

        return 0;
        */


    }

    public static boolean stopAtClosestInterval(double robotHeading, Telemetry telemetry)
    {
        List<Double> allPossibleValues = Arrays.asList(0d, 45d, 90d, 135d, 180d, -45d, -90d, -135d);
        if(45 - Math.abs(robotHeading % 45 ) <= .5d || Math.abs(robotHeading % 45) <= .5d)
        {
            return false;
        }
        else return true;
    }
}
