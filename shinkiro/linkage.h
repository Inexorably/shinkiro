#pragma once

#include <iostream>
#include <vector>

#include "math.h"

#include "globals.h"

#include "Eigen/Dense"

namespace shinkiro {
	//Class defining individual links of a linkage.  Assumes units are consistent.
	class Link {
		public:
			double m_length;		//Link length.
			double m_radius;		//Distance along the link from the link origin to the center of mass.
			double m_inertia;
			double m_mass;
			double m_theta;			//Current angle in rad.
			double m_omega;			//Current angular velocity in rad/s.
			double m_alpha;			//Current angular acceleration in rad/s.
			
			//These values can either be set by user for forward dynamics or found and set by inverse dynamics function.
			double m_forceX;
			double m_forceY;
			double m_torque;		//Ccw positive.

			//Some useful functions for simplifying calculation code.
			double f_cta() const;		//Returns cos(theta)*alpha.
			double f_sta() const;		//Returns sin(theta)*alpha.
			double f_ctw2() const;		//Returns cos(theta)*omega^2.
			double f_stw2() const;		//Returns sin(theta)*omega^2.

			//Default constructor, primarily useful for testing so we fill with some arbitrary values.
			Link();
	};

	class Linkage {
		public:
			std::vector<Link> m_links;

			//Use inverse dynamics to calculate the forces and moments for a three link planar bipedal / inverted pendulum model.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a vector of the forces and torques of order [Fx1; Fx2; Fy1; Fy2; T1; T2; Fx3; Fy3; T3].
			Eigen::VectorXd f_inverseDynamics();

			//Use forward dynamics to find the angular accelerations resulting from some given forces and torques.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a vector of angular accelerations of order [alpha1; alpha2; alpha3].
			Eigen::VectorXd f_forwardDynamics();


			//Default constructor, primarily useful for testing so we fill with some arbitrary values.  Defaults to 3 link linkage.
			Linkage();
	};


}

