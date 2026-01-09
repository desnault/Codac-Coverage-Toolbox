/*
 * @file example2_covered_area_uncertain_range_and_fov.cpp
 * @brief Example of computing the covered area using SepDynPolarProj with a three-valued perception sensor.
 *
 * This example shows how to:
 * 1. Define a 3D trajectory for the vehicle (x, y, heading).
 * 2. Convert the trajectory into a TubeVector for interval-based projection.
 * 3. Define a detection area with a main range, an uncertain border, and a field-of-view (FOV) in the robot frame.
 * 4. Use SepDynPolarProj to project the sensor coverage along the trajectory.
 * 5. Apply a paving algorithm (SIVIA) to display the covered area.
 * 6. Display the vehicle trajectory, tube, sensor coverage, and vehicle positions at selected times.
 *
 * Notes on robot frame and angles:
 * - Robot frame: x points forward, y points to the left, heading is rotation around z-axis.
 * - delta intervals define the sensor's angular FOV relative to the current heading.
 * - 0 corresponds to forward direction; positive angles rotate counterclockwise.
 * - SepDynPolarProj handles angle wrap-around using modulo2pi to ensure the FOV stays within [-pi, pi].
 * - Three-valued perception:
 *      - "range" defines the area always detected (certain coverage).
 *      - "uncertain_range" and "uncertain_fov" define the area possibly detected (uncertain coverage).
 *      - Together, they allow modeling of sensors with uncertain detection near the boundaries.
 */

#include "codac.h"
#include "vibes.h"

#include "SepDynPolarProj.h"

#include <vector>
#include <iostream>

using namespace codac;



/*==================================================================================
 * TRAJECTORY SETUP
 *==================================================================================*/

const double t0 = 0.55;        // Initial time
const double tf = 12;          // Final time
const double dt = 0.01;        // Time step
const Interval tdomain(t0, tf); // Time domain

// Trajectory function (x, y, heading)
TFunction path("( 5*cos(0.5*t)*sin(t) ; 5*cos(0.5*t)*cos(t) ; atan2(-2.5*sin(0.5*t)*cos(t) - 5*cos(0.5*t)*sin(t) , -2.5*sin(0.5*t)*sin(t) + 5*cos(0.5*t)*cos(t)))");

// Generate TrajectoryVector from the path
TrajectoryVector generate_trajectory()
{
    TrajectoryVector traj(tdomain, path, dt);
    traj[2] = traj[2].make_continuous(); // Ensure heading continuity
    return traj;
}

// Convert TrajectoryVector to TubeVector
TubeVector generate_tube(TrajectoryVector &traj)
{
    TubeVector tube(traj, dt);
    return tube;
}



/*==================================================================================
 * DETECTION PARAMETERS
 *==================================================================================*/

// Main detection range (certain coverage)
Interval detection_range(1, 2);

// Uncertain detection range (uncertain coverage border)
Interval detection_border(0.5, 2.5);

// Sensor field-of-view (FOV) in robot frame
Interval fov = Interval(0).inflate(M_PI/4.);

// Uncertain FOV for the uncertain border
Interval uncertain_fov = Interval(0).inflate(1.2 * M_PI/4.);



/*==================================================================================
 * PROJECTION PARAMETERS
 *==================================================================================*/

Interval t_proj(t0, tf); // Project along entire trajectory
double eps_proj = 0.01;  // Temporal resolution for projection



/*==================================================================================
 * PAVING PARAMETERS
 *==================================================================================*/

double paving_resolution = 0.25;       // Paving resolution
IntervalVector paving_area = {{-7, 7}, {-8, 8}}; // Area to pave



int main()
{
    // Generate trajectory and TubeVector
    TrajectoryVector my_traj = generate_trajectory();
    TubeVector my_tube = generate_tube(my_traj);

    // Initialize VIBes drawing
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(100, 100, 800, 800);
    fig.axis_limits(-9, 9, -9, 9);
    fig.draw_box(paving_area, "black");

    // Create the SepDynPolarProj separator/projector
    // Models three-valued sensor coverage: certain + uncertain areas
    SepDynPolarProj my_sep(my_tube, detection_range, detection_border, fov, uncertain_fov, t_proj, eps_proj, true);

    // Apply SIVIA paving algorithm
    // Classifies area as detected, uncertain, or not detected
    SIVIA(paving_area, my_sep, paving_resolution);

    // Draw trajectory and tube
    fig.add_trajectory(&my_traj, "x", 0, 1, "black");
    fig.add_tube(&my_tube, "x", 0, 1);

    // Display vehicle and sensor coverage at selected times
    std::vector<double> detection_time = {1.3, 6.5, 11.5};
    for (int i = 0; i < detection_time.size(); i++)
    {
        // Get robot position and heading at detection time
        double px = my_traj[0](detection_time[i]);
        double py = my_traj[1](detection_time[i]);
        double heading = my_traj[2](detection_time[i]);

        // Draw the certain coverage (range + FOV)
        fig.draw_pie(px, py, detection_range, heading + fov, "black");

        // Draw the uncertain coverage border
        fig.draw_pie(px, py, detection_border, heading + uncertain_fov, "black");

        // Draw vehicle at this time
        fig.draw_vehicle(detection_time[i], &my_traj, 0.6);
    }

    // Show figure without vehicle at the end
    fig.show(0);

    // End drawing
    vibes::endDrawing();

	return EXIT_SUCCESS;
}