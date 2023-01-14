/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is for storing Cartesian coordinates and converting them to polar.
 */
public class Point2d {
    private double x_;
    private double y_; 
    public Point2d(double x, double y){
        x_ = x;
        y_ = y;
    }
    public double getX(){
        return x_;
    }
    public double getY(){
        return y_;
    }
    public void updateValues(double x, double y){
        x_ = x;
        y_ = y;
    }
    public void updateValues(Point2d newPoint){
        x_ = newPoint.getX();
        y_ = newPoint.getY();
    }
    public static PolarPoint2d getPolarPoint(Point2d point){
        double r = Math.sqrt((point.x_ * point.x_) + (point.y_ * point.y_));
        double p = Math.atan2(point.y_, point.x_);
        return new PolarPoint2d(r,p);
    }
    public void transformBy(double x, double y){
        x_ += x;
        y_ += y;
    }
    public String toString(){
        return "X: " + x_ + " Y: " + y_;
    }
    public static Point2d fromPose(Pose2d pose){
        Translation2d translation = pose.getTranslation();
        return new Point2d(translation.getX(), translation.getY());
    }
    public void rotateBy(Rotation2d rotation){
        PolarPoint2d polarPoint = getPolarPoint(this);
        polarPoint.transformBy(0, rotation);
        updateValues(PolarPoint2d.getCartesianPoint(polarPoint));
    }
}
