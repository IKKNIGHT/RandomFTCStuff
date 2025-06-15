package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

public class IPLSmoothner {
    private double gamma;
    private double delta;

    public IPLSmoothner(double gamma, double delta) {
        this.gamma = gamma;
        this.delta = delta;
    }

    public IPLSmoothner() {
        this(1.0, 1.0); // Safe defaults
    }

    /**
     * The following method calculates the filtered value given a list of previous values. Using the formula as follows:
     * <br>
     * <img src="../imgs/IPLA.png/" />
     * <br>
     * Where S is calculated as follows:
     * <br>
     * <img src="../imgs/IPLB.png/" />
     * <br>
     * Where N is the size of the window, t is the current time index, P is the list of previous values.
     * @param P a list of previous values
     * @param timeIndex the index of the current value in the list
     * @param windowSize the size of the window
     * @return the filtered value
     */
    public double getFilteredValue(List<Double> P, int timeIndex, int windowSize) {
        List<Double> weights = new ArrayList<>();
        List<Double> weightedValues = new ArrayList<>();

        for (int k = 0; k <= windowSize; k++) {
            double denominator = (Math.pow((k + 2),gamma) * (Math.pow(Math.log(k + 3),delta)));
            // Avoid divide by zero
            if (denominator == 0) {
                denominator = 1e-6;
            }
            double weight = 1.0 / denominator;

            weights.add(weight);
            weightedValues.add(weight * P.get(timeIndex - k));
        }

        double sumWeights = sum(weights);
        double sumWeightedValues = sum(weightedValues);

        return sumWeightedValues / sumWeights;
    }

    private double sum(List<Double> list) {
        double total = 0;
        for (double val : list) {
            total += val;
        }
        return total;
    }

    // Getters and setters for tuning
    public void setGamma(double gamma) {
        this.gamma = gamma;
    }

    public void setDelta(double delta) {
        this.delta = delta;
    }

    public double getGamma() {
        return gamma;
    }

    public double getDelta() {
        return delta;
    }
}