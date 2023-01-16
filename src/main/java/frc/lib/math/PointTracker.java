package frc.lib.math;

import java.util.Arrays;

import frc.lib.util.Util;

public class PointTracker {
    private double[] r, p, x, y;
    private final int keep;

    /**
     * @param keep the number of history values to keep for processing
     */
    public PointTracker(int keep){
        this.keep = keep;
        r = new double[keep];
        p = new double[keep];
        x = new double[keep];
        y = new double[keep];
        for(int i = 0; i < keep; i++){
            r[i] = 0;
            p[i] = 0;
            x[i] = 0;
            y[i] = 0;
        }
    }

    public double p(){
        return p[p.length - 1];
    }

    public double r(){
        return r[r.length - 1];
    }

    public double x(){
        return x[x.length - 1];
    }

    public double y(){
        return y[y.length - 1];
    }

    public PointTracker pr(double p, double r){
        this.p = Util.shiftLeft(this.p, p);
        this.r = Util.shiftLeft(this.r, r);
        computeCartesian();
        return this;
    }

    public PointTracker xy(double x, double y){
        this.x = Util.shiftLeft(this.x, x);
        this.y = Util.shiftLeft(this.y, y);
        computePolar();
        return this;
    }

    public PointTracker getFuture(int steps){
        double xPrediction = Util.last(Sequencer.predict(Sequencer.compute(x), steps), 0);
        double yPrediction = Util.last(Sequencer.predict(Sequencer.compute(y), steps), 0);
        double[] newx = Util.shiftLeft(x, xPrediction);
        double[] newy = Util.shiftLeft(y, yPrediction);
        PointTracker result = new PointTracker(keep);
        for (int i = 0; i < keep; i++){
            result.xy(newx[i], newy[i]);
        }
        return result;
    }

    private void computePolar(){
        r = Util.shiftLeft(r, Math.sqrt((x() * x()) + (y() * y())));
        p = Util.shiftLeft(p, Math.atan2(y(), x()));
    }

    private void computeCartesian(){
        x = Util.shiftLeft(x, r() * Math.cos(p()));
        y = Util.shiftLeft(y, r() * Math.sin(p()));
    }

    public String toString(){
        return "PointTracker:\n\tx:" 
                + x() + 
                " y:" + y() + 
                "\n\tp:" + p() + 
                " r:" + r() + 
                "\n\thistory - x:" + Arrays.toString(x) + " y:" + Arrays.toString(y);
    }

    public static void main(String[] args){
        PointTracker test = new PointTracker(3);
        test.xy(0,0);
        test.xy(1,1);
        test.xy(2,3);
        test.xy(3,5);
        test.xy(4,7);
        test.xy(5,10);
        test.xy(6,5);
        test.xy(7, 10);
        System.out.println(test);
        System.out.println(test.getFuture(2));
    }
}
