public class PID {

    // The proportional, integral, derivative, and feedforward constants for the PID controller
    double kP;
    double kI;
    double kD;
    double kF;

    // A counter for the integral term of the PID controller
    double integralCounter = 0;

    // The time of the last update and the error at the last update
    long lastUpdateTime = -1L;
    double lastUpdateError = -1D;

    // A constructor for the PID controller, taking in the proportional, integral, derivative, and feedforward constants as arguments
    public PID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    private double[] getNumericalValues(double currentState, double targetState) {

        // Calculate the current update error as the difference between the target state and the current state
        double currentUpdateError = targetState - currentState;

        // Calculate the change in error as the difference between the current update error and the last update error
        long deltaTime = this.lastUpdateTime == -1 ? 0 : System.currentTimeMillis() - lastUpdateTime;
        double changeInError = this.lastUpdateError == -1 ? 0 : currentUpdateError - this.lastUpdateError;

        // Calculate the proportion, integral, and derivative terms for a PID (proportional-integral-derivative) controller
        double proportion = currentUpdateError * kP;
        double derivative = changeInError / deltaTime;
        this.integralCounter += currentUpdateError * deltaTime;

        // Store the current time and current update error for use in the next iteration
        this.lastUpdateTime = System.currentTimeMillis();
        this.lastUpdateError = currentUpdateError;

        // Return an array containing the proportion, integral, derivative, and feedforward terms
        return new double [] {
                proportion,
                this.integralCounter * this.kI,
                derivative * this.kD,
                this.kF
        };
    }

    public double getOutputFromError(double targetState, double currentState) {
        // Call the getNumericalValues method, passing in the current state and target state as arguments
        double[] numericalValues = getNumericalValues(currentState, targetState);

        // Initialize a sum variable to 0
        double sum = 0;

        // Iterate through the numerical values array
        for (double numericalValue : numericalValues) {
            // Add the current numerical value to the sum
            sum += numericalValue;
        }

        // If the sum is greater than 1, set it to 1
        if (sum > 0.8) {
            sum = 0.8;
        }
        // If the sum is less than -1, set it to -1
        else if (sum < -0.8) {
            sum = -0.8;
        }

        // Return the sum
        return sum;
    }
}