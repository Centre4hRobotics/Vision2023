package frc.robot;

import edu.wpi.first.math.*;



public class CC4HTriangulationImplementation {
    private class CC4HInternalPolarVector {
        double range;
        double theta;
    };

    private class CC4HInternalCartesianVector {
        double x;
        double y;
    };

    private class CC4HInternalVxVyInfo {
        CC4HInternalCartesianVector First_AprilTagLocalPosition;
        CC4HInternalCartesianVector First_AprilTagAbsolutePosition;
        CC4HInternalCartesianVector Second_AprilTagLocalPosition;
        CC4HInternalCartesianVector Second_AprilTagAbsolutePosition;
    };

    /*
     * Converts polar vector to cartesian vector.
     * Programmer's note: Probably a more object-oriented way to do this >_>
     * 
     * @param polar     Polar vector to convert
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector convertPolarToCartesian(CC4HInternalPolarVector polar) {

    }

    /*
     * Solves for Vx.
     * 
     * @param VxSolverParameter     Parameters to feed into the solver.
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector VxSolver(CC4HInternalVxVyInfo VxSolverParameters) {

    }

    /*
     * Solves for Vy.
     * 
     * @param VySolverParameter     Parameters to feed into the solver.
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector VySolver(CC4HInternalVxVyInfo VySolverParameters) {

    }

    /*
     * 
     */
    double AngleBetweenAbsoluteYRelativeYSolver(CC4HInternalCartesianVector VyVector) {
        return 1.0;
    }

    /*
     * 
     */
    CC4HTriangulationImplementation.TriangulationOutputInfo FinalizeTriangularOperation(TriangulationInputInfo inputParams, double AngleBetweenAbsoluteYRelativeY) {

    }



    public class TriangulationInputInfo {
        double AprilTagPosition_x;
        double AprilTagPosition_y;
        double AprilTagYaw;
        double RobotYaw;
    };

    public class TriangulationOutputInfo {
        double xResult;
        double yResult;
    };

    /*
     * The one function to kick the whole process off.
     * 
     * @param TriangulationParams       The input information to the triangulation system
     */
    CC4HTriangulationImplementation.TriangulationOutputInfo calculateTriangulationVector(TriangulationInputInfo FirstTriangulationParams, TriangulationInputInfo SecondTriangulationInfo) {

    }
}
