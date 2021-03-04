#pragma once

#include <vector>

namespace shinkiro {
	//Class defining individual links of a linkage.  Assumes units are consistent.
	class link {
		public:
			double m_length;		//Link length.
			double m_radius;		//Distance along the link from the link origin to the center of mass.
			double m_intertia;
			double m_mass;
			double m_theta;		//The current link angle in radians.
			
			//These values can either be set by user for forward dynamics or found and set by inverse dynamics function.
			double m_forceX;
			double m_forceY;
			double m_torque;		//Ccw positive.
	};

	class linkage {
		public:
			std::vector<link> m_links;

			//Use inverse dynamics to calculate the forces and moments for a three link planar bipedal / inverted pendulum model.
			//Assumes foot is massless and remains grounded for period of analysis.
			//Returns a linkage with the m_forceX, m_forceY, and m_torque values of each link found.
			linkage inverseDynamics(const linkage l);



	};


}

