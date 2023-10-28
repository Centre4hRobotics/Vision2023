package frc.robot;

import edu.wpi.first.math.*;
import java.lang.Math;


public class CC4HTriangulationImplementation {
    public class CC4HInternalPolarVector {
        double r;
        double theta; //if theta is the angle from the forward vector the calculations become easier, we use that here

        public CC4HInternalPolarVector(double radius, double angle) {
            r = radius;
            theta = angle;
        }
    };

    public class CC4HInternalCartesianVector {
        double x;
        double y;

        public CC4HInternalCartesianVector(double xval, double yval) {
            x = xval;
            y = yval;
        }
    };

    private class CC4HInternalVxVyInfo {
        CC4HInternalCartesianVector firstAprilTagLocalPosition;
        CC4HInternalCartesianVector firstAprilTagAbsolutePosition;
        CC4HInternalCartesianVector secondAprilTagLocalPosition;
        CC4HInternalCartesianVector secondAprilTagAbsolutePosition;

        public CC4HInternalVxVyInfo(CC4HInternalCartesianVector firstLocalPosition, TriangulationInputInfo firstAprilTagInput, CC4HInternalCartesianVector secondLocalPosition, TriangulationInputInfo secondAprilTagInput) {
            firstAprilTagLocalPosition = firstLocalPosition;
            firstAprilTagAbsolutePosition = firstAprilTagInput.aprilTagAbsolutePosition;
            secondAprilTagLocalPosition = secondLocalPosition;
            secondAprilTagAbsolutePosition = secondAprilTagInput.aprilTagAbsolutePosition;
        }
    };

    /*
     * Converts polar vector to cartesian vector.
     * Programmer's note: Probably a more object-oriented way to do this >_>
     * 
     * @param polar     Polar vector to convert
     */
    private CC4HTriangulationImplementation.CC4HInternalCartesianVector convertPolarToCartesian(CC4HInternalPolarVector polar) {
        CC4HInternalCartesianVector cartesian = new CC4HInternalCartesianVector( polar.r * Math.sin( polar.theta ) , polar.r * Math.cos( polar.theta ) );
        return cartesian;
    }

    /*
     * Solves for Vx (the unit vector moving rightwards relative to the robot) in absolute coordinates.
     * 
     * @param VxSolverParameter     Parameters to feed into the solver.
     */
    private CC4HTriangulationImplementation.CC4HInternalCartesianVector VxSolver(CC4HInternalVxVyInfo VxSolverParameters) {
        double a1 = VxSolverParameters.firstAprilTagLocalPosition.x;
        double b1 = VxSolverParameters.firstAprilTagLocalPosition.y;
        double a2 = VxSolverParameters.secondAprilTagLocalPosition.x;
        double b2 = VxSolverParameters.secondAprilTagLocalPosition.y;

        double x1 = VxSolverParameters.firstAprilTagAbsolutePosition.x;
        double y1 = VxSolverParameters.firstAprilTagAbsolutePosition.y;
        double x2 = VxSolverParameters.secondAprilTagAbsolutePosition.x;
        double y2 = VxSolverParameters.secondAprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vx = new CC4HInternalCartesianVector( (b1*x2-b2*x1)/(b1*a2-b2*a1) , (b1*y2-b2*y1)/(b1*a2-b2*a1) );
        return Vx;
    }

    /*
     * Solves for Vy (the unit vector moving forward relative to the robot) in absolute coordinates.
     * 
     * @param VySolverParameter     Parameters to feed into the solver.
     */
    private CC4HTriangulationImplementation.CC4HInternalCartesianVector VySolver(CC4HInternalVxVyInfo VySolverParameters) {
        double a1 = VySolverParameters.firstAprilTagLocalPosition.x;
        double b1 = VySolverParameters.firstAprilTagLocalPosition.y;
        double a2 = VySolverParameters.secondAprilTagLocalPosition.x;
        double b2 = VySolverParameters.secondAprilTagLocalPosition.y;

        double x1 = VySolverParameters.firstAprilTagAbsolutePosition.x;
        double y1 = VySolverParameters.firstAprilTagAbsolutePosition.y;
        double x2 = VySolverParameters.secondAprilTagAbsolutePosition.x;
        double y2 = VySolverParameters.secondAprilTagAbsolutePosition.y;

        CC4HInternalCartesianVector Vy = new CC4HInternalCartesianVector( (a1*x2-a2*x1)/(a1*b2-a2*b1) , (a1*y2-a2*y1)/(a1*b2-a2*b1) );
        return Vy;
    }

    /*
     * Finds the angle between the absolute vertical unit vector (0,1) and the relative vertical unit vector Vy.
     * 
     * @param Vy     The forward vector Vy relative to the robot in absolute coordinates.
     */
    private double angleBetweenAbsoluteYRelativeYSolver(CC4HInternalCartesianVector Vy) {
        // arccos( ([0 1] · Vy) / (|[0 1]| |Vy|) ) = arccos(Vy.y / |Vy|)
        double length = Math.sqrt(Vy.x*Vy.x + Vy.y*Vy.y);
        return Math.acos(Vy.y / length);
    }

    public class TriangulationInputInfo {
        CC4HInternalCartesianVector aprilTagAbsolutePosition;
        CC4HInternalPolarVector aprilTagDistanceAndYaw;

        public TriangulationInputInfo(CC4HInternalCartesianVector absolute, CC4HInternalPolarVector distanceyaw) {
            aprilTagAbsolutePosition = absolute;
            aprilTagDistanceAndYaw = distanceyaw;
        }
    };

    public class TriangulationOutputInfo {
        double xResult;
        double yResult;

        public TriangulationOutputInfo(double xval, double yval) {
            xResult = xval;
            yResult = yval;
        }
    };

    /*
     * The one function to kick the whole process off.
     * 
     * @param firstTriangulationInfo       The input information for the first AprilTag.
     * @param secondTriangulationInfo       The input information for the second AprilTag.
     */
    public CC4HTriangulationImplementation.TriangulationOutputInfo calculateTriangulationVector(TriangulationInputInfo firstTriangulationInfo, TriangulationInputInfo secondTriangulationInfo) {
        CC4HInternalCartesianVector firstAprilTagLocalCartesian = convertPolarToCartesian(firstTriangulationInfo.aprilTagDistanceAndYaw);
        CC4HInternalCartesianVector secondAprilTagLocalCartesian = convertPolarToCartesian(secondTriangulationInfo.aprilTagDistanceAndYaw);
        CC4HInternalVxVyInfo VxVySolvingVectors = new CC4HInternalVxVyInfo(firstAprilTagLocalCartesian, firstTriangulationInfo, secondAprilTagLocalCartesian, secondTriangulationInfo);
        
        CC4HInternalCartesianVector Vx = VxSolver(VxVySolvingVectors);
        CC4HInternalCartesianVector Vy = VySolver(VxVySolvingVectors);
        
        double theta = angleBetweenAbsoluteYRelativeYSolver(Vy);
        double x1 = firstTriangulationInfo.aprilTagAbsolutePosition.x;
        double y1 = firstTriangulationInfo.aprilTagAbsolutePosition.y;
        double d = firstTriangulationInfo.aprilTagDistanceAndYaw.r;
        double alpha = firstTriangulationInfo.aprilTagDistanceAndYaw.theta;

        TriangulationOutputInfo robotPosition = new TriangulationOutputInfo(x1 - d * Math.sin(theta + alpha),  y1 - d * Math.cos(theta + alpha));
        return robotPosition;
    }
}
