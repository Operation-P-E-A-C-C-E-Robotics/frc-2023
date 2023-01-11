package frc.lib.math;

public class NiceCurve {
    private double mult;
    private double c;
    private double b;
    private double a;
    private double m1;
    private double m2;

    public NiceCurve(double a, double m1, double b, double m2, double c){
        this.a = a;
        this.b = b;
        this.c = c;
        this.m1 = m1;
        this.m2 = m2;
    }

    public double get(double x){
        // x *= mult;
        boolean neg = x < 0;
        x = Math.abs(x);
        x = Math.pow(x*m1, a) + Math.pow(x*m2, b);
        x = Math.sin(x);
        x = Math.pow(x, c);
        return neg ? -x : x;
    }

    public static NiceCurve preset1(){
        return new NiceCurve(1, 0.5, 6.4, 1.0, 1.0);
    }

    public static void main(String args[]){
        NiceCurve test = NiceCurve.preset1();

        System.out.println(test.get(-1));
    }
}
