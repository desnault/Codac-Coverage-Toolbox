/*
 * @file SepDynDiskProj.cpp
 * @author Damien ESNAULT (PhD student, ENSTA/Lab-STICC)
 * @date 2025
 *
 * @brief
 * Implementation of SepDynDiskProj, a high-level class for projecting
 * the disk-shaped coverage of a moving object along a trajectory using a ranging sensor.
 *
 * This class combines the SepDynDisk separator with the SepProj projection,
 * making it easy for beginners to compute coverage over a trajectory.
 *
 * See SepDynDiskProj.h for full class and constructor documentation.
 */

#include "SepDynDiskProj.h"

namespace 
{
    // -------------------------------
    // Helper functions for input checks
    // -------------------------------

    // Check that the projection time interval is within the trajectory time domain
    const Interval verify_t_proj(const Interval t_proj, const TubeVector& traj)
    {
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(..., const Interval& t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        return t_proj;
    }
    
    // Check that the projection precision is positive
    const double verify_eps_proj(const double eps_proj)
    {
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(..., const Interval& t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }
        
        return eps_proj;
    }

    // -------------------------------
    // Functions to generate SepDynDisk separators
    // -------------------------------

    // Binary coverage constructor (disk)
    SepDynDisk generate_sep(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check detection radius
        if (range<=0)
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double range must be greater than 0");
        }

        // Create and return the separator
        return SepDynDisk(traj, range, true);
    }

    // Three-valued coverage constructor (disk + uncertain border)
    SepDynDisk generate_sep2(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Check trajectory dimension
        if (traj.size()<2)
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning):  The input TubeVector traj must have a dimension greater than or equal to 2");
        }
        if (traj.size()>2 && !overpass_warning)
        {
            std::cerr << "[WARNING] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 2, only the first two components will be used" << std::endl;
        }

        // Check detection radius
        if (range<=0)
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double range must be greater than 0");
        }

        // Check uncertain border width
        if (border_width<0)
        {
            throw std::range_error("[ERROR] SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double border_width must be greater than or equal to 0");
        }

        // Create and return the separator
        return SepDynDisk(traj, range, border_width, true);
    }
       
}

// -------------------------------
// Constructor definitions
// -------------------------------

// Binary coverage (disk)
SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const Interval t_proj, const double eps_proj, bool overpass_warning): 
_sep(generate_sep(traj, range, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj,traj), verify_eps_proj(eps_proj)) 
{}

// Three-valued coverage (disk + uncertain border)
SepDynDiskProj::SepDynDiskProj(const TubeVector& traj, const double range, const double border_width, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(generate_sep2(traj, range, border_width, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, verify_t_proj(t_proj, traj), verify_eps_proj(eps_proj)) 
{}