#pragma once

#include <iostream>
#include <vector>

#include "math.h"

#include "Eigen/Dense"

namespace shinkiro {
	//Class defining individual links of a linkage.  Assumes units are consistent.
	class Link {
		public:
			double m_length;		//Link length.
			double m_radius;		//Distance along the link from the link origin to the center of mass.
			double m_intertia;
			double m_mass;
			double m_theta;			//Current angle in rad.
			double m_omega;			//Current angular velocity in rad/s.
			double m_alpha;			//Current angular acceleration in rad/s.
			
			//These values can either be set by user for forward dynamics or found and set by inverse dynamics function.
			double m_forceX;
			double m_forceY;
			double m_torque;		//Ccw positive.

			//Some useful functions for simplifying calculation code.
			double f_sin() const;		//Return sin(theta), which commonly appears in calculations.
			double f_cos() const;		//Return cos(theta), which commonly appears in calculations.

			//Default constructor, primarily useful for testing so we fill with some arbitrary values.
			Link();
	};

	class Linkage {
		public:
			std::vector<Link> m_links;

			//Use inverse dynamics to calculate the forces and moments for a three link planar bipedal / inverted pendulum model.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Sets the force and torque values of m_links[i].m_forceX etc.
			Linkage f_inverseDynamics();


			//Default constructor, primarily useful for testing so we fill with some arbitrary values.  Defaults to 3 link linkage.
			Linkage();
	};


}

