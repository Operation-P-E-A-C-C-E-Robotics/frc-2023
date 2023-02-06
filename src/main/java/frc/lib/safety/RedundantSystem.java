package frc.lib.safety;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.Supplier;

public class RedundantSystem <T> {
    private final Supplier<T> primary;
    private final Supplier<T>[] secondary;
    private final ValueCheck<T> valueCheck;
    private boolean failed = false;
    private boolean average = false;
    private int currentSecondary = 0;

    @SafeVarargs
    public RedundantSystem(ValueCheck<T> valueCheck, Supplier<T> primary, Supplier<T>... secondary){
        this.primary = primary;
        this.secondary = secondary;
        this.valueCheck = valueCheck;
    }

    public void fallback(){
        failed = true;
    }

    public T get(){
        selfCheck();
        if(failed) return secondary[currentSecondary].get();
        return primary.get();
    }

    public static interface ValueCheck <T> {
        public boolean check(T value, boolean primary);
    }

    public void selfCheck(){
        if(!valueCheck.check(get(), failed)) {
            if(failed) {
                currentSecondary++;
                if (currentSecondary >= secondary.length) currentSecondary = 0;
            } else {
                failed = true;
            }
        }
    }

    public static class SlewRateCheck implements ValueCheck<Double> {
        private final SlewRateLimiter rateLimiter;

        public SlewRateCheck(double maxRate) {
            rateLimiter = new SlewRateLimiter(maxRate);
        }

        public boolean check(Double value, boolean failed) {
            return rateLimiter.calculate(value) == value;
        }
    }

    public static class RangeCheck implements ValueCheck<Double> {
        private double min = 0;
        private double max = 0;

        public RangeCheck(double min, double max) {
            this.min = min;
            this.max = max;
        }

        public boolean check(Double value, boolean failed) {
            return value >= min && value <= max;
        }
    }

    public static class AverageCheck implements ValueCheck<Double> {
        private final double deviation;
        private final LinearFilter averageFilter;

        public AverageCheck(int samples, double deviation) {
            averageFilter = LinearFilter.movingAverage(samples);
            this.deviation = deviation;
        }

        public boolean check(Double value, boolean failed) {
            return Math.abs(value - averageFilter.calculate(value)) <= deviation;
        }
    }

    public static class DiscrepancyCheck implements ValueCheck<Double> {

        private final Supplier<Double>[] suppliers;
        private final double deviation;

        @SafeVarargs
        public DiscrepancyCheck(double deviation, Supplier<Double>...suppliers){
            this.suppliers = suppliers;
            this.deviation = deviation;
        }

        public boolean check(Double value, boolean failed) {
            var average = 0.0;
            for(Supplier<Double> supplier : suppliers){
                average += supplier.get();
            }
            average /= suppliers.length;
            for (Supplier<Double> supplier : suppliers) {
                if (Math.abs(supplier.get() - average) > deviation) return false;
            }
            return true;
        }
    }

    public static class MultiCheck implements ValueCheck<Double> {
        private final ValueCheck<Double>[] checks;

        @SafeVarargs
        public MultiCheck(ValueCheck<Double>... checks) {
            this.checks = checks;
        }

        public boolean check(Double value, boolean failed) {
            for (ValueCheck<Double> check : checks) {
                if (!check.check(value, failed)) return false;
            }
            return true;
        }
    }
}
