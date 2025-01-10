package org.firstinspires.ftc.teamcode.utils.math;

public class Vector3D {

    private double x, y, z;
    private double angle,magnitude;

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        angle =getAngle();
        magnitude =getMagnitude();
    }

    public Vector3D(double x, double y) {
        this(x, y, 0);
        angle =getAngle();
        magnitude =getMagnitude();
    }





    public Vector3D() {
        this(0, 0, 0);
    }

    public Vector3D(Vector3D other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    public static Vector3D fromAngleAndMagnitude(double angle, double m) {
        return new Vector3D(m * Math.cos(angle), m * Math.sin(angle));
    }


    public double getMagnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }
    public double getAngle(){
        return Math.atan2(y, x);
    }

    public Vector3D plus(Vector3D other) {
        return new Vector3D(x + other.x, y + other.y, z + other.z);
    }

    public void scaleToMagnitude(double targetMagnitude) {
        double currentMagnitude = getMagnitude();
        scaleBy(1.0 / currentMagnitude);
        scaleBy(targetMagnitude);
    }

    public void scaleBy(double a) {
        x *= a;
        y *= a;
        z *= a;
    }
    public Vector3D scaledBy(double a) {
        return new Vector3D(x * a, y * a, z * a);
    }

    public Vector3D scaledToMagnitude(double targetMagnitude) {
        Vector3D aux = new Vector3D(this);
        aux.scaleToMagnitude(targetMagnitude);
        return aux;
    }

    public static Vector3D rotateBy(Vector3D vector, double theta) {
        return new Vector3D(
                Math.cos(theta) * vector.getX() + Math.sin(theta) * vector.getY(),
                Math.cos(theta) * vector.getY() - Math.sin(theta) * vector.getX()
        );
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    @Override
    public String toString() {
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(z);
    }

}