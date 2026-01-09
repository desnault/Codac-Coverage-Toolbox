/*
 * @file SepDynPolarProj.h
 * @author Damien ESNAULT (PhD student, ENSTA / Lab-STICC)
 * @date 2025
 *
 * @brief
 * SepDynPolarProj is a high-level class designed for beginners to compute
 * the coverage area of a moving vehicle equipped with an oriented range sensor.
 *
 * The perception model combines:
 *  - a distance constraint (range),
 *  - an angular constraint (field of view, FOV),
 *  - and the vehicle heading along a known trajectory.
 *
 * The covered area corresponds to a polar sector (or annular sector)
 * attached to the vehicle and moving with it over time.
 *
 * This class integrates:
 *  - the SepDynPolar separator (local space-time consistency),
 *  - with the SepProj projection (integration over time),
 * into a single easy-to-use object.
 *
 * Beginners can directly use SepDynPolarProj to compute the area covered
 * by an oriented sensor along a trajectory, without manipulating low-level
 * separators or projection operators.
 *
 * Advanced users can manually combine SepDynPolar with SepProj
 * for finer control.
 *
 * The class supports:
 *  - binary coverage (inside / outside),
 *  - three-valued coverage (inside / uncertain / outside),
 * depending on the constructor used.
 */

#ifndef __SEP_DYN_POLAR_PROJ_H___
#define __SEP_DYN_POLAR_PROJ_H___

#include "codac.h"
#include "SepDynPolar.h"

using namespace codac;

/*
 * @class SepDynPolarProj
 * @brief
 * High-level wrapper for projecting the detection area of a moving vehicle
 * equipped with an oriented range sensor.
 *
 * The vehicle trajectory must contain at least:
 *  - x-position,
 *  - y-position,
 *  - heading (orientation).
 *
 * Robot frame and angle convention (VERY IMPORTANT):
 *
 * At each time t, the vehicle defines a local robot frame:
 *  - origin   : (x(t), y(t))
 *  - x-axis   : forward direction of the vehicle
 *  - y-axis   : left direction
 *
 * The heading θ(t) is the orientation of the robot frame
 * with respect to the global frame, and angles follow the
 * standard mathematical convention:
 *  - angles increase counterclockwise,
 *  - θ = 0 means facing the global +x direction.
 *
 * The field-of-view angle interval `delta` is defined in the ROBOT FRAME:
 *  - delta = 0 corresponds to the forward direction,
 *  - positive angles correspond to the left side,
 *  - negative angles correspond to the right side.
 *
 * At time t, the actual detection angles in the WORLD FRAME are:
 *      θ(t) + delta
 *
 * Examples:
 *  - delta = [-π/4, π/4] : forward-looking sensor (90° FOV)
 *  - delta = [π/2, π]   : left-facing sensor
 *  - delta = [-π, π]    : omnidirectional angular coverage
 *
 * IMPORTANT:
 *  - delta should normally be defined in [-π, π]
 *  - the class internally applies a modulo 2π operation
 *    as a safety mechanism for beginner users
 *
 * This class performs runtime checks and prints warnings
 * when parameters are inconsistent or suboptimal.
 */
class SepDynPolarProj : public SepProj
{
    public:
        
        /*
         * @brief Binary coverage constructor (inside / outside)
         *
         * @param traj The TubeVector representing the vehicle trajectory.
         *             The trajectory must have dimension >= 3:
         *             (x, y, heading, ...).
         * @param range Detection range [min, max] (subset of [0, +∞)).
         * @param delta Field-of-view angle interval (in the robot frame).
         * @param t_proj Time interval over which the coverage is projected.
         * @param eps_proj Projection precision (must be > 0).
         * @param overpass_warning If true, disables runtime warnings.
         *
         * This constructor corresponds to binary coverage:
         *  - inside  : certainly detected,
         *  - outside : certainly not detected.
         *
         * Warnings:
         *  - If traj has more than 3 dimensions, only (x, y, heading) are used.
         *  - If range.lb() = 0, better performance can be achieved using
         *    SepDynPieProj.
         *  - If the width of delta ≥ 2π, the sensor becomes omnidirectional
         *    and SepDynDistProj is more appropriate.
         */
        SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Three-valued coverage constructor (inside / uncertain / outside)
         *
         * @param traj The TubeVector representing the vehicle trajectory
         *             (x, y, heading, ...).
         * @param range Detection range where detection is certain.
         * @param uncertain_range Extended range modeling uncertainty.
         * @param delta Field-of-view angle interval (certain detection).
         * @param uncertain_delta Extended angular interval modeling uncertainty.
         * @param t_proj Time interval over which the coverage is projected.
         * @param eps_proj Projection precision (must be > 0).
         * @param overpass_warning If true, disables runtime warnings.
         *
         * The following must hold:
         *  - range ⊆ uncertain_range
         *  - delta ⊆ uncertain_delta (after modulo 2π)
         *
         * This constructor enables three-valued logic:
         *  - inside    : certainly detected,
         *  - uncertain : possibly detected,
         *  - outside   : certainly not detected.
         */
        SepDynPolarProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval delta, const Interval uncertain_delta, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
    
    private:

        // Internal separator used for projection, fully managed by this class
        SepDynPolar _sep;
};

# endif