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
 * This class is to store polar points (Like the ones given by our limelight), 
 * convert them to Cartesian, and transform them (to hit the inner goal).
 */
public class PolarPoint2d {
    private double r_; //radial coordinate
    private double p_; //angular coordinate
    public PolarPoint2d(double r, double p){
        r_ = r;
        p_ = p;
    }
    public PolarPoint2d(double r, Rotation2d p){
        r_ = r;
        p_ = p.getRadians();
    }
    public double getR(){
        return r_;
    }
    public double getP(){
        return p_;
    }
    public void updateValues(PolarPoint2d newPoint){
        r_ = newPoint.getR();
        p_ = newPoint.getP();
    }
    public void updateValues(double r, double p){
        r_ = r;
        p_ = p;
    }
    public Rotation2d getRotation2dP(){
        return new Rotation2d(p_);
    }
    public static Point2d getCartesianPoint(PolarPoint2d point){
        double x = point.r_ * Math.cos(point.p_);
        double y = point.r_ * Math.sin(point.p_);
        return new Point2d(x,y);
    }
    public void transformBy(double r, double p){
        r_ += r;
        p_ += p;
    }
    public void transformBy(double r, Rotation2d p){
        r_ += r;
        p_ += p.getRadians();
    }
    public void cartesianTransform(double transformX, double transformY){
        Point2d cartesianPoint = getCartesianPoint(this);
        cartesianPoint.transformBy(transformX,transformY);
        updateValues(Point2d.getPolarPoint(cartesianPoint));
    }
    public static PolarPoint2d fromPose(Pose2d pose){
        Translation2d translation = pose.getTranslation();
        Point2d cartesian = new Point2d(translation.getX(), translation.getY());
        return Point2d.getPolarPoint(cartesian);
    }
    public String toString(){
        return "R: " + r_ + " P: " + p_;
    }
}