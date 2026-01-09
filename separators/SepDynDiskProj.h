/*
 * @file SepDynDiskProj.h
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 * 
 * @brief 
 * SepDynDiskProj is a high-level class designed for beginners to compute
 * the coverage area of a moving object along a trajectory using a disk-shaped
 * ranging sensor.
 * 
 * @details
 * This class integrates the SepDynDisk separator with the SepProj projection
 * in a single object, making it easy to compute projected coverage over a trajectory.
 * 
 * It supports both:
 *  - Binary coverage (inside/outside disk)
 *  - Three-valued coverage (inside/uncertain/outside disk), where the uncertain
 *    border is added around the disk.
 * 
 * Beginners can simply create an instance of SepDynDiskProj, providing the
 * trajectory, sensor parameters, and projection parameters. The class handles
 * runtime checks and warnings automatically.
 * 
 * Advanced users can bypass this class and manually combine SepDynDisk with
 * SepProj for more control.
 * 
 * The class assumes 2D trajectories, but will accept higher-dimensional TubeVectors.
 * Only the first two components are used for coverage computation.
*/

#ifndef __SEP_DYN_DISK_PROJ_H___
#define __SEP_DYN_DISK_PROJ_H___

#include "codac.h"
#include "SepDynDisk.h"

using namespace codac;

/*
 * @class SepDynDiskProj
 * @brief 
 * High-level wrapper for projecting the disk-shaped detection area of a moving object
 * along a trajectory using a ranging sensor.
 *
 * The class performs runtime checks and prints warnings or errors if the input
 * trajectory, sensor range, or other parameters are not valid. This helps beginners
 * quickly identify misuses and understand how to properly initialize the class.
 *
 * Supported coverage types:
 *  - Binary coverage: inside/outside disk
 *      * Constructor using `range`
 *  - Three-valued coverage: inside/uncertain/outside disk
 *      * Constructor using `range` + `border_width`
*/
class SepDynDiskProj : public SepProj
{
    public:

        /*
         * @brief Binary coverage constructor (inside/outside disk)
         * @param traj The TubeVector representing the trajectory (dimension >= 2)
         * @param range Detection radius (disk radius > 0)
         * @param t_proj Time interval over which to project the coverage (subset of traj.tdomain())
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning Disable warnings if true
         *
         * Checks:
         *  - traj must have dimension >= 2
         *  - range must be > 0
         *  - t_proj must be subset of traj.tdomain()
         *
         * Warnings:
         *  - If traj has more than 2 dimensions, only the first two are used
        */
        SepDynDiskProj(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
        
        /*
         * @brief Three-valued coverage constructor (inside/uncertain/outside disk)
         * @param traj The TubeVector representing the trajectory (dimension >= 2)
         * @param range Detection radius (disk radius > 0)
         * @param border_width Width of the uncertain border around the disk (>= 0)
         * @param t_proj Time interval over which to project the coverage (subset of traj.tdomain())
         * @param eps_proj Precision of the projection (must be > 0)
         * @param overpass_warning Disable warnings if true
         *
         * Checks:
         *  - traj must have dimension >= 2
         *  - range must be > 0
         *  - border_width >= 0
         *  - t_proj must be subset of traj.tdomain()
         *
         * Warnings:
         *  - If traj has more than 2 dimensions, only the first two are used
         *  - If border_width is too large compared to range, the uncertain border
         *    is truncated at 0
        */
        SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning = false);
    
    private:

        // Internal separator used for projection, fully managed by this class
        SepDynDisk _sep;
};

# endif