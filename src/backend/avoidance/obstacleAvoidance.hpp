#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include "../bridge/bridge.hpp"
#include "../math/matrix.hpp"
#include "cmath"
#include "vector"

using namespace std;

// Constants declarations
const double OBSTACLE_DEGREE = 0.01;
const double LAMBDA = 5.0;
const double INFLUENCE = 120.0;
const double BODY_RADIUS = 12.0;
const double MARGIN = 12.0;
const double MIN_DIST = 2.0;
const double REPULSIVE_GAIN = 55.0;
const double TANGENT_GAIN = 0.25;
const double STATIC_DIRECTION_GAIN = 0.25;
const double DIRECTION_ORDER = 1.0;

// Get all joint positions from current angles
vector<Matriks> getJointPositions(vector<double>& lengths, vector<double>& angles) {
    vector<Matriks> joints;
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;

    joints.push_back(Matriks::point2D(x, y));
    for (int i = 0; i < lengths.size(); i++) {
        angle += angles[i];
        x += cos(angle) * lengths[i];
        y += sin(angle) * lengths[i];
        joints.push_back(Matriks::point2D(x, y));
    }

    return joints;
}

// Get the nearest point on a link segment from an obstacle
Matriks nearestPointOnSegment(Matriks& a, Matriks& b, Obstacle& o) {
    double sx = Matriks::coordinateX(b) - Matriks::coordinateX(a);
    double sy = Matriks::coordinateY(b) - Matriks::coordinateY(a);
    double len2 = sx * sx + sy * sy;

    if (len2 < 1e-12) {
        return a;
    }

    double t = ((Matriks::coordinateX(o.midPoint) - Matriks::coordinateX(a)) * sx + (Matriks::coordinateY(o.midPoint) - Matriks::coordinateY(a)) * sy) / len2;
    if (t < 0.0) {
        t = 0.0;
    }

    if (t > 1.0) {
        t = 1.0;
    }

    return Matriks::point2D(Matriks::coordinateX(a) + t * sx, Matriks::coordinateY(a) + t * sy);
}

// Compute obstacle avoidance angle delta for all links
Matriks getObstacleAvoidanceDelta(vector<double>& lengths, vector<double>& angles, Matriks target, vector<Obstacle>& objects, vector<int>& tangentSigns) {
    int jointCount = lengths.size();
    Matriks total(jointCount, 1);

    if (objects.empty()) {
        return total;
    }

    if (tangentSigns.size() != objects.size()) {
        tangentSigns = vector<int>(objects.size(), 0);
    }

    vector<Matriks> joints = getJointPositions(lengths, angles);
    double tangentGain = TANGENT_GAIN;

    // Check every obstacle against every link
    for (int i = 0; i < objects.size(); i++) {
        for (int j = 0; j < jointCount; j++) {
            Matriks nearest = nearestPointOnSegment(joints[j], joints[j + 1], objects[i]);
            double dist = Matriks::distance(nearest, objects[i].midPoint) - objects[i].radius - BODY_RADIUS - MARGIN;

            // Ignore obstacle if it is outside the influence distance
            if (dist >= INFLUENCE) {
                continue;
            }
            if (dist < MIN_DIST) {
                dist = MIN_DIST;
            }

            Matriks away = Matriks::normalize(Matriks::point2D(Matriks::coordinateX(nearest) - Matriks::coordinateX(objects[i].midPoint), Matriks::coordinateY(nearest) - Matriks::coordinateY(objects[i].midPoint)));
            Matriks toTarget = Matriks::normalize(Matriks::point2D(Matriks::coordinateX(target) - Matriks::coordinateX(nearest), Matriks::coordinateY(target) - Matriks::coordinateY(nearest) ));

            // Keep one tangential direction to reduce oscillation
            if (tangentSigns[i] == 0) {
                Matriks test = Matriks::point2D(-Matriks::coordinateY(away), Matriks::coordinateX(away));
                if (Matriks::dotProduct(test, toTarget) < 0.0) {
                    tangentSigns[i] = -1;
                } else {
                    tangentSigns[i] = 1;
                }
            }

            // Static velocity-direction factor from the paper
            Matriks toObstacle = Matriks::point2D(-Matriks::coordinateX(away), -Matriks::coordinateY(away));
            double directionDot = Matriks::dotProduct(toObstacle, toTarget);
            if (directionDot < -1.0) {
                directionDot = -1.0;
            }

            if (directionDot > 1.0) {
                directionDot = 1.0;
            }

            double thetaRobTarget = acos(directionDot);
            double sigmaStatic = STATIC_DIRECTION_GAIN * thetaRobTarget;
            double directionFactor = exp(DIRECTION_ORDER * sigmaStatic);
            double mag = REPULSIVE_GAIN * directionFactor * (1.0 / dist - 1.0 / INFLUENCE) / pow(dist, 2.0);
            Matriks tangent = Matriks::point2D(-Matriks::coordinateY(away) * tangentSigns[i], Matriks::coordinateX(away) * tangentSigns[i]);
            Matriks velocity = Matriks::point2D(mag * (Matriks::coordinateX(away) + Matriks::coordinateX(tangent) * tangentGain), mag * (Matriks::coordinateY(away) + Matriks::coordinateY(tangent) * tangentGain));

            // Create the Jacobian Matriks
            vector<vector<double>> jacobianData(2, vector<double>(j + 1, 0.0));
            Matriks current = joints[j + 1];
            for (int k = 0; k <= j; k++) {
                angles[k] += OBSTACLE_DEGREE;
                Matriks moved = getJointPositions(lengths, angles)[j + 1];
                angles[k] -= OBSTACLE_DEGREE;

                // Approximate the partial derivative of the link position
                jacobianData[0][k] = (Matriks::coordinateX(moved) - Matriks::coordinateX(current)) / OBSTACLE_DEGREE;
                jacobianData[1][k] = (Matriks::coordinateY(moved) - Matriks::coordinateY(current)) / OBSTACLE_DEGREE;
            }

            // Using Damped Least Squares / Levenberg-Marquardt
            Matriks J(jacobianData);
            Matriks JT = J;
            JT.transpose();

            Matriks JJT = J * JT;
            Matriks lambda2I = Matriks::identity(JJT.getRow());
            for (int i = 0; i < lambda2I.getRow(); i++) {
                for (int j = 0; j < lambda2I.getColumn(); j++) {
                    lambda2I.getData()[i][j] *= LAMBDA * LAMBDA;
                }
            }

            Matriks A(JJT.getRow(), JJT.getColumn());
            for (int i = 0; i < JJT.getRow(); i++) {
                for (int j = 0; j < JJT.getColumn(); j++) {
                    A.getData()[i][j] += (JJT.getData()[i][j] + lambda2I.getData()[i][j]);
                }
            }

            Matriks AInvers = A.inverse();
            Matriks AInversVelocity = AInvers * velocity;
            Matriks delta = JT * AInversVelocity;

            // Apply link delta only to joints that affect the current link
            for (int k = 0; k <= j; k++) {
                total.getData()[k][0] += delta.getData()[k][0];
            }
        }
    }

    return total;
}

// Compute the smallest distance between robot body and obstacles
double minimumClearance(vector<double>& lengths, vector<double>& angles, vector<Obstacle>& objects) {
    vector<Matriks> joints = getJointPositions(lengths, angles);
    double best = 1e9;

    for (int i = 0; i < objects.size(); i++) {
        for (int j = 0; j < lengths.size(); j++) {
            Matriks nearest = nearestPointOnSegment(joints[j], joints[j + 1], objects[i]);
            double clearance = Matriks::distance(nearest, objects[i].midPoint) - objects[i].radius - BODY_RADIUS;
            if (clearance < best) {
                best = clearance;
            }
        }
    }

    return best;
}

// Select the first delta that does not collide with obstacles
Matriks selectHardConstrainedDelta(vector<double>& lengths, vector<double>& angles, Matriks& targetDelta, Matriks& obstacleDelta, vector<double>& minAngles, vector<double>& maxAngles, vector<Obstacle>& objects, double obstacleWeight, double maxDelta) {
    int jointCount = lengths.size();
    Matriks zero(jointCount, 1);
    Matriks best = zero;
    
    if (objects.empty()) {
        Matriks delta(jointCount, 1);
        for (int i = 0; i < jointCount; i++) {
            delta.getData()[i][0] = targetDelta.getData()[i][0];
            if (delta.getData()[i][0] < -maxDelta) {
                delta.getData()[i][0] = -maxDelta;
            }
            if (delta.getData()[i][0] > maxDelta) {
                delta.getData()[i][0] = maxDelta;
            }
        }

        return delta;
    }

    vector<double> targetScales = {1.0, 0.7, 0.4, 0.2, 0.0};
    vector<double> obstacleScales = {1.0, 1.5, 2.5, 4.0, 6.0};
    vector<double> stepScales = {1.0, 0.75, 0.5, 0.25, 0.1, 0.05};
    double currentClearance = minimumClearance(lengths, angles, objects);
    double bestClearance = currentClearance;

    // Try several target and obstacle weights before applying the result
    for (double targetScale : targetScales) {
        for (double obstacleScale : obstacleScales) {
            for (double stepScale : stepScales) {
                Matriks candidate(jointCount, 1);
                vector<double> candidateAngles = angles;

                for (int i = 0; i < jointCount; i++) {
                    double value = targetScale * targetDelta.getData()[i][0] + obstacleScale * obstacleWeight * obstacleDelta.getData()[i][0];
                    candidate.getData()[i][0] = stepScale * value;
                    if (candidate.getData()[i][0] < -maxDelta) {
                        candidate.getData()[i][0] = -maxDelta;
                    }
                    if (candidate.getData()[i][0] > maxDelta) {
                        candidate.getData()[i][0] = maxDelta;
                    }

                    candidateAngles[i] += candidate.getData()[i][0];
                    if (candidateAngles[i] < minAngles[i]) {
                        candidateAngles[i] = minAngles[i];
                    }
                    if (candidateAngles[i] > maxAngles[i]) {
                        candidateAngles[i] = maxAngles[i];
                    }
                }

                double clearance = minimumClearance(lengths, candidateAngles, objects);

                // Reject new collision from a safe position
                if (clearance < 0.0 && currentClearance >= 0.0) {
                    continue;
                }
                if (clearance >= 0.0) {
                    return candidate;
                }
                if (clearance > bestClearance) {
                    bestClearance = clearance;
                    best = candidate;
                }
            }
        }
    }

    if (currentClearance < 0.0) {
        return best;
    } else {
        return zero;
    }
}

#endif
