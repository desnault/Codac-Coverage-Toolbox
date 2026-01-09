/*
 * @file SepDynPieProj.cpp
 * @brief Implementation of SepDynPieProj, a high-level class for projecting
 *        the coverage of a moving vehicle equipped with an oriented range sensor.
 *
 * This class combines the SepDynPie separator with the SepProj projection, 
 * making it easy for beginners to compute pie-shaped coverage along a trajectory.
 *
 * See SepDynPieProj.h for full class and constructor documentation.
 */

#include "SepDynPieProj.h"

namespace 
{
    // -------------------------------
    // Helper function to safely wrap angles
    // -------------------------------

    /*
     * @brief Ensure an interval angle is expressed in [-π, π] range
     *
     * This function is used as a safety mechanism for beginners.
     * Users might define delta intervals exceeding 2π or outside [-π, π].
     *
     * - If delta width ≥ 2π, returns [-π, π] as full coverage.
     * - Otherwise, it shifts the interval to be centered within [-π, π].
     *
     * @param delta Input angular interval (in radians, robot frame)
     * @return Interval adjusted modulo 2π
     */
    Interval modulo2pi(const Interval delta)
    {
        // If the field of view spans the full circle (or more),
        // angular constraints are meaningless
        if (delta.ub()-delta.lb()>=2*M_PI)
        {
            // Return the full angular domain [-π, π]
            return (Interval(0)|Interval(Interval::TWO_PI)) - Interval::PI;
        }
        
        // Otherwise, ensure that the interval mean lies within [-π, π]
        Interval mod_delta(delta);
        double mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;

        // Shift interval by multiples of 2π until mean is within bounds
        while(mean_angle>M_PI)
        {
            mod_delta = mod_delta - 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }
        
        // Shift interval by multiples of -2π until mean is within bounds
        while(mean_angle<-M_PI)
        {
            mod_delta = mod_delta + 2*M_PI;
            mean_angle = (mod_delta.ub()+mod_delta.lb())/2.;
        }

        // Return the interval delta modulo 2π and centered on [-π, π]
        return mod_delta;
    }

    // -------------------------------
    // Verify input and generate SepDynPie separator
    // -------------------------------

    /*
     * @brief Generate a binary coverage separator (inside/outside)
     */
    SepDynPie verify_and_generate_sep(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning)
    {
        // Trajectory dimension must be at least 3: (x, y, heading)
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPie::SepDynPie(const TubeVector& traj, const double range, const double fov, const double fov_offset, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Projection interval must be subset of trajectory time
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        // Projection epsilon must be positive
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning): The input double eps_proj must be greater than 0");
        }

        // Range must be positive
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify FOV angle
        double fov_angle = fov;
        if (fov_angle<0)
        {
            // Ensure FOV is positive
            fov_angle = -1.*fov_angle; 
        }
        if (fov_angle>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning): You initialized an instance of class SepDynPieProj with a FOV superior or equal to 2*PI. Please use the SepDynDiskProj class for better performances."<<std::endl;
            }
            // Limit FOV angle to 2PI
            fov_angle = 2*M_PI;
        }

        // Wrap FOV offset into [-π, π]
        double fov_offset_angle = fov_offset;
        while (fov_offset_angle>M_PI)
        {
            fov_offset_angle = fov_offset_angle - 2*M_PI;
        }
        while (fov_offset_angle<-M_PI)
        {
            fov_offset_angle = fov_offset_angle + 2*M_PI;
        }
        
        return SepDynPie(traj, range, fov_angle, fov_offset_angle, true);
    }

    /*
     * @brief Generate a binary coverage separator (delta-based)
     */
    SepDynPie verify_and_generate_sep2(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Verify trajectory
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify t_proj
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        // Verify eps_proj
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }

        // Verify range
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double range must be greater than 0");
        }

        // Wrap delta interval to [-π, π]
        Interval mod_delta = modulo2pi(delta);
        if (mod_delta.ub()-mod_delta.lb()>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPieproj with an Interval delta with a width superior or equal to 2PI. Please use the SepDynDiskProj class for better performances"<<std::endl;
            }
        }
        
        return SepDynPie(traj, range, mod_delta, true);
    }

    /*
     * @brief Generate a three-valued coverage separator (inside/uncertain/outside)
     */
    SepDynPie verify_and_generate_sep3(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning)
    {
        // Verify trajectory
        if (traj.size()<3)
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj must have a dimension greater than or equal to 3");
        }
        if (traj.size()>3 && !overpass_warning)
        {
            std::cerr<< "[WARNING] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input TubeVector traj has a dimension greater than 3, only the first three components will be used" <<std::endl;
        }

        // Verify t_proj
        if (!t_proj.is_subset(traj.tdomain()))
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval t_proj must be a subset of the trajectory time domain");
        }

        // Verify eps_proj
        if (eps_proj<=0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double eps_proj must be greater than 0");
        }

        // Verify range
        if (range <= 0) 
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double range must be greater than 0");
        }

        // Verify range uncertain border
        if (range_border<0)
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input double range_border must be greater than or equal to 0");
        }

        // Verify delta width (check that the width is inferior to 2PI => No overlap)
        Interval mod_delta = modulo2pi(delta);
        if (mod_delta.ub()-mod_delta.lb()>=2*M_PI)
        {
            if (!overpass_warning)
            {
                std::cerr<<"[WARNING] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): You initialized an instance of class SepDynPieproj with an Interval delta with a width superior or equal to 2PI. Please use the SepDynDiskProj class for better performances"<<std::endl;
            }
        }

        // Verify delta border width
        Interval mod_delta_border = modulo2pi(delta_border);
        

        // Check that delta is a subset of delta_borders
        if (!mod_delta.is_subset(mod_delta_border))
        {
            throw std::range_error("[ERROR] SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning): The input Interval delta must be a subset of the Interval delta_border");
        }
        
        return SepDynPie(traj, range, range_border, mod_delta, mod_delta_border, true);
    }
}

// -------------------------------
// Constructor definitions
// -------------------------------

SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double fov, const Interval t_proj, const double eps_proj, const double fov_offset, bool overpass_warning):
_sep(verify_and_generate_sep(traj, range, fov, t_proj, eps_proj, fov_offset, overpass_warning)), 
SepProj(_sep, t_proj, eps_proj) 
{}

SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const Interval delta, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(verify_and_generate_sep2(traj, range, delta, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, t_proj, eps_proj) 
{}

SepDynPieProj::SepDynPieProj(const TubeVector& traj, const double range, const double range_border, const Interval delta, const Interval delta_border, const Interval t_proj, const double eps_proj, bool overpass_warning):
_sep(verify_and_generate_sep3(traj, range, range_border, delta, delta_border, t_proj, eps_proj, overpass_warning)), 
SepProj(_sep, t_proj, eps_proj) 
{}