package frc.robot;

import edu.wpi.first.math.*;
import java.lang.Math;


public class CC4HTriangulationImplementation {
    private class CC4HInternalPolarVector {
        double r;
        double theta; //if theta is the angle from the forward vector the calculations become easier
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

        public CC4HInternalVxVyInfo(CC4HInternalCartesianVector FirstAprilTagLocalPosition, TriangulationInputInfo FirstAprilTagInput, CC4HInternalCartesianVector SecondAprilTagLocalPosition, TriangulationInputInfo SecondAprilTagInput) {
            First_AprilTagLocalPosition = FirstAprilTagLocalPosition;
            First_AprilTagAbsolutePosition = FirstAprilTagInput.AprilTagAbsolutePosition;
            Second_AprilTagLocalPosition = SecondAprilTagLocalPosition;
            Second_AprilTagAbsolutePosition = SecondAprilTagInput.AprilTagAbsolutePosition;
        }
    };

    /*
     * Converts polar vector to cartesian vector.
     * Programmer's note: Probably a more object-oriented way to do this >_>
     * 
     * @param polar     Polar vector to convert
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector convertPolarToCartesian(CC4HInternalPolarVector polar) {
        CC4HInternalCartesianVector cartesian = new CC4HInternalCartesianVector();
        cartesian.x = polar.r * Math.sin( polar.theta ); //does the wpi library have a cos/sin function to use instead?
        cartesian.y = polar.r * Math.cos( polar.theta );
        return cartesian;
    }

    /*
     * Solves for Vx (the unit vector moving rightwards relative to the robot) in absolute coordinates.
     * 
     * @param VxSolverParameter     Parameters to feed into the solver.
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector VxSolver(CC4HInternalVxVyInfo VxSolverParameters) {
        double a1 = VxSolverParameters.First_AprilTagLocalPosition.x;
        double b1 = VxSolverParameters.First_AprilTagLocalPosition.y;
        double a2 = VxSolverParameters.Second_AprilTagLocalPosition.x;
        double b2 = VxSolverParameters.Second_AprilTagLocalPosition.y;

        double x1 = VxSolverParameters.First_AprilTagAbsolutePosition.x;
        double y1 = VxSolverParameters.First_AprilTagAbsolutePosition.y;
        double x2 = VxSolverParameters.Second_AprilTagAbsolutePosition.x;
        double y2 = VxSolverParameters.Second_AprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vx = new CC4HInternalCartesianVector();
        Vx.x = (b1*x2-b2*x1)/(b1*a2-b2*a1);
        Vx.y = (b1*y2-b2*y1)/(b1*a2-b2*a1);
        return Vx;
    }

    /*
     * Solves for Vy (the unit vector moving forward relative to the robot) in absolute coordinates.
     * 
     * @param VySolverParameter     Parameters to feed into the solver.
     */
    CC4HTriangulationImplementation.CC4HInternalCartesianVector VySolver(CC4HInternalVxVyInfo VySolverParameters) {
        double a1 = VxSolverParameters.First_AprilTagLocalPosition.x;
        double b1 = VxSolverParameters.First_AprilTagLocalPosition.y;
        double a2 = VxSolverParameters.Second_AprilTagLocalPosition.x;
        double b2 = VxSolverParameters.Second_AprilTagLocalPosition.y;

        double x1 = VxSolverParameters.First_AprilTagAbsolutePosition.x;
        double y1 = VxSolverParameters.First_AprilTagAbsolutePosition.y;
        double x2 = VxSolverParameters.Second_AprilTagAbsolutePosition.x;
        double y2 = VxSolverParameters.Second_AprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vy = new CC4HInternalCartesianVector();
        Vy.x = (a1*x2-a2*x1)/(a1*b2-a2*b1);
        Vy.y = (a1*y2-a2*y1)/(a1*b2-a2*b1);
        return Vy;
    }

    /*
     * Finds the angle between the absolute vertical unit vector (0,1) and the relative vertical unit vector Vy.
     * 
     * @param Vy     The forward vector Vy relative to the robot in absolute coordinates.
     */
    double AngleBetweenAbsoluteYRelativeYSolver(CC4HInternalCartesianVector Vy) {
        // arccos( ([0 1] · Vy) / (|[0 1]| |Vy|) ) = arccos(Vy.y / |Vy|)
        double length = Math.sqrt(Vy.x**2 + Vy.y**2);
        return Math.arccos(Vy.y / length);
    }

    public class TriangulationInputInfo {
        CC4HInternalCartesianVector AprilTagAbsolutePosition;
        CC4HInternalPolarVector AprilTagDistanceAndYaw;
    };

    public class TriangulationOutputInfo {
        double xResult;
        double yResult;
    };

    /*
     * The one function to kick the whole process off.
     * 
     * @param FirstTriangulationInfo       The input information for the first AprilTag.
     * @param SecondTriangulationInfo       The input information for the second AprilTag.
     */
    CC4HTriangulationImplementation.TriangulationOutputInfo calculateTriangulationVector(TriangulationInputInfo FirstTriangulationInfo, TriangulationInputInfo SecondTriangulationInfo) {
        CC4HInternalCartesianVector FirstAprilTagLocalCartesian = convertPolarToCartesian(FirstTriangulationInfo.AprilTagDistanceAndYaw);
        CC4HInternalCartesianVector SecondAprilTagLocalCartesian = convertPolarToCartesian(SecondTriangulationInfo.AprilTagDistanceAndYaw);
        CC4HInternalVxVyInfo VxVySolvingVectors = new CC4HInternalVxVyInfo(FirstAprilTagLocalCartesian, FirstTriangulationInfo, SecondAprilTagLocalCartesian, SecondTriangulationInfo);
        
        CC4HInternalCartesianVector Vx = VxSolver(VxVySolvingVectors);
        CC4HInternalCartesianVector Vy = VySolver(VxVySolvingVectors);
        
        double theta = AngleBetweenAbsoluteYRelativeYSolver(Vy);
        double x1 = VxSolverParameters.First_AprilTagAbsolutePosition.x;
        double y1 = VxSolverParameters.First_AprilTagAbsolutePosition.y;
        double d = FirstTriangulationInfo.AprilTagDistanceAndYaw.r;
        double alpha = FirstTriangulationInfo.AprilTagDistanceAndYaw.theta;

        CC4HInternalCartesianVector robotPosition = new CC4HInternalCartesianVector();
        robotPosition.x = x1 - d * Math.sin(theta + alpha);
        robotPosition.x = y1 - d * Math.cos(theta + alpha);
        return robotPosition;
    }
}
