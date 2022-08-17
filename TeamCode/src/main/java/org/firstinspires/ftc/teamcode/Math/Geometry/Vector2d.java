package org.firstinspires.ftc.teamcode.Math.Geometry;

/**
 * Think of a vector as a ray with a starting point at the origin.
 * The point represented by (x, y) is the point that the ray's arrow points.
 * This represents the vector in 2d Cartesian space. A vector is essentially a value,
 * or magnitude, with a set direction. You can also put a vector in polar form:
 * (r, theta), where r is the magnitude and theta is the directional angle.
 */
public class Vector2d {

    private final double m_x;
    private final double m_y;

    /**
     * Default constructor, no params.
     * Initializes to x and y components of 0
     */
    public Vector2d() {
        this(0, 0);
    }

    /**
     * The constructor that sets up the x and y values for the 2d vector.
     *
     * @param x the x value of the vector
     * @param y the y value of the vector
     */
    public Vector2d(double x, double y) {
        m_x = x;
        m_y = y;
    }

    public Vector2d(Vector2d other) {
        m_x = other.getX();
        m_y = other.getY();
    }

    /**
     * Rotate the vector in Cartesian space.
     *
     * @param angle angle in degrees by which to rotate vector counter-clockwise.
     */
    public Vector2d rotateBy(double angle) {
        angle = Math.toRadians(angle);
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double x = m_x * cosA - m_y * sinA;
        double y = m_x * sinA + m_y * cosA;
        return new Vector2d(x,y);
    }

    /**
     * @return the angle of the vector
     */
    public double angle() { return Math.atan2(m_y, m_x); }

    /**
     * adds two vectors in 2d space and returns a new vector of the sum
     * @param other the vector to add
     * @return the sum of the vectors
     */
    public Vector2d plus(Vector2d other){
        return new Vector2d(m_x + other.m_x, m_y + other.m_y);
    }
    /**
     * subtracts two vectors in 2d space and returns a new vector of the difference
     * @param other the vector to subtract
     * @return the difference of the vectors
     */
    public Vector2d minus(Vector2d other){
        return plus(other.unaryMinus());
    }

    /**
     * returns the inverse of the vector, equivalent to rotating by 180 degrees
     * @return the inverse of the current vector
     */
    public Vector2d unaryMinus() {
        return new Vector2d(-m_x, -m_y);
    }

    /**
     *
     * @param scalar the value to multiple the vector by
     * @return the new scaled vector
     */
    public Vector2d times(double scalar) {
        return new Vector2d(m_x * scalar, m_y * scalar);
    }

    /**
     *
     * @param scalar the value to divide the vector by
     * @return the new scaled vector
     */
    public Vector2d div(double scalar) {
        return new Vector2d(m_x / scalar, m_y / scalar);
    }

    /**
     * the x component of the vector
     * @return the x component of the vector
     */
    public double getX() {return m_x;}
    /**
     * the y component of the vector
     * @return the y component of the vector
     */
    public double getY() {return m_y;}

    /**
     * Returns dot product of this vector with argument.
     *
     * @param other Vector with which to perform dot product.
     */
    public double dot(Vector2d other) {
        return m_x * other.m_x + m_y * other.m_y;
    }

    /**
     * Returns magnitude (norm) of the vector.
     */
    public double magnitude() {
        return Math.hypot(m_x,m_y);
    }

    /**
     * Returns scalar projection of this vector onto another vector.
     *
     * @param other Vector onto which to project this vector.
     */
    public double scalarProject(Vector2d other) {
        return this.dot(other) / other.magnitude();
    }

    /**
     * Scales the values of the vector by some scalar
     *
     * @param scalar the scalar
     */
    public Vector2d scale(double scalar) {
        return new Vector2d(m_x * scalar, m_y * scalar);
    }

    /**
     * Projects this vector onto another vector
     *
     * @param other the other vector
     */
    public Vector2d project(Vector2d other) {
        return other.scale(dot(other) / (magnitude() * other.magnitude()));
    }

    /**
     * Checks equality between this Vector2d and another object
     *
     * @param obj the other object
     * @return whether the two objects are equal
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector2d) {
            return Math.abs(((Vector2d) obj).getX() - m_x) < 1E-9
                    && Math.abs(((Vector2d) obj).getY() - m_y) < 1E-9;
        }
        return false;
    }

    @Override
    public String toString() {
        String x = Double.toString(m_x);
        String y = Double.toString(m_y);
        return "( " + x + ", " + y + " )";
    }
}