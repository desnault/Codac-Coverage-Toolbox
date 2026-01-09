/*
 * @file SepDynDistProj.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 * 
 * @brief 
 * SepDynDistProj is a high-level class designed for beginners to compute
 * the coverage area of a moving object along a trajectory using a ranging sensor.
 * 
 * The coverage modeled corresponds to the perception set supported by the
 * SepDynDist class: either an annulus or a disk around the object. 
 * This means SepDynDistProj is not generic for any arbitrary perception set,
 * but specifically tailored for sensor-based distance coverage.
 * 
 * This class integrates the SepDynDist separator with the SepProj projection 
 * in a single object, making it easy to compute projected coverage over a trajectory.
 * 
 * Beginners can simply create an instance of SepDynDistProj, providing the
 * trajectory and sensor parameters, and the class will handle all checks,
 * warnings, and the projection automatically.
 * 
 * Advanced users can bypass this class and manually combine SepDynDist with
 * SepProj if they want more control.
 * 
 * The class supports both binary coverage (inside/outside) and three-valued 
 * coverage (inside/uncertain/outside) depending on which constructor is used.
*/

#ifndef __SEP_DYN_DIST_PROJ_H___
#define __SEP_DYN_DIST_PROJ_H___

#include "codac.h"
#include "SepDynDist.h"

using namespace codac;

/*
 * @class SepDynDistProj
 * @brief 
 * High-level wrapper for projecting the detection area of a moving object
 * along a trajectory using a ranging sensor (disk or annulus coverage).
 *
 * The class performs runtime checks and prints warnings or errors if the input
 * trajectory, sensor range, or other parameters are not valid. This helps beginners
 * quickly identify misuses and understand how to properly initialize the class.
 *
 * Supported coverage types:
 *  - Binary coverage: inside/outside detection.
 *      * Constructors using `r_in`/`r_out` or `range`
 *  - Three-valued coverage: inside/uncertain/outside detection.
 *      * Constructors using `range`/`border_width` or `range`/`uncertain_range`
*/
class SepDynDistProj : public SepProj
{
    public:
        /*
         * @brief Binary coverage constructor (inside/outside)
         * @param traj The TubeVector representing the trajectory
         * @param r_in Inner detection radius (distance always detected)
         * @param r_out Outer detection radius (distance possibly detected)
         * @param t_proj Time interval over which to project the coverage
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning If true, disables warnings for trajectory size or radius choices
         *
         * This constructor corresponds to the SepDynDist constructor with r_in/r_out.
         * Checks:
         *  - traj must have dimension >= 2
         *  - r_in >= 0 and r_in <= r_out
         *  - t_proj must be within the trajectory time domain
         * Warnings:
         *  - If traj has more than 2 dimensions, only the first two are used
         *  - If r_in = 0, better performance is achieved using SepDynDiskProj
        */
        SepDynDistProj(const TubeVector& traj, const double r_in, const double r_out, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Binary coverage constructor (inside/outside) using a single range
         * @param traj The TubeVector representing the trajectory
         * @param range Detection range [min,max] (subset of [0,+inf])
         * @param t_proj Time interval over which to project the coverage
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning Disable warnings if true
         *
         * This constructor corresponds to the core SepDynDist constructor using a range.
         * Used for binary coverage (inside/outside) with a ranging sensor.
        */
        SepDynDistProj(const TubeVector& traj, const Interval range, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Three-valued coverage constructor (inside/uncertain/outside)
         * @param traj The TubeVector representing the trajectory
         * @param range Detection range [min,max] (subset of [0,+inf])
         * @param border_width Width of the uncertain detection border
         * @param t_proj Time interval over which to project the coverage
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning Disable warnings if true
         *
         * This constructor corresponds to SepDynDist with a range and uncertain border.
         * Used for three-valued coverage (inside/uncertain/outside) with a ranging sensor.
        */
        SepDynDistProj(const TubeVector& traj, const Interval range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Three-valued coverage constructor (inside/uncertain/outside)
         * @param traj The TubeVector representing the trajectory
         * @param range Detection range [min,max] (subset of [0,+inf])
         * @param uncertain_range Interval representing uncertain detection [min,max]
         * @param t_proj Time interval over which to project the coverage
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning Disable warnings if true
         *
         * This is the core three-valued coverage constructor for beginners.
         * The range must be a subset of the uncertain_range.
         * Used for three-valued coverage (inside/uncertain/outside) with a ranging sensor.
        */
        SepDynDistProj(const TubeVector& traj, const Interval range, const Interval uncertain_range, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
    
    private:
        // Internal separator used for projection, fully managed by this class
        SepDynDist _sep;
};

# endif