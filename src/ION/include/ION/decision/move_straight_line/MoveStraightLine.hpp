#pragma once

#include <armadillo>

namespace ION{
	namespace decision{
		namespace move_straight_line{
			/**
			 * Calculate a unit direction vector based on a given compass
			 * angle.
			 */
			arma::vec direction_vector_from_north(double angle);
			/**
			 * Calculate the angle between the north direction vector (1, 0)
			 * and the vector provided.
			 */
			double angle_from_north(const arma::vec& direction);
			
			/**
			 * Normalises and angle in the range [0, 2pi] to an angle in
			 * the range [-pi, pi] (so commands dicate efficient turning
			 * angles)
			 */
			double normalise_turning_angle(double angle);

			/**
			 * For describing robot state - points are in default robot
			 * coordinate system.
			 */
			struct State{
				/// 2D unit vector describing where the robot is facing
				arma::vec direction;
				/// 2D robot position (units in m.)
				arma::vec position;
			};
	
			/**
			 * Describes move commands as as this library sees them.
			 * All coordinates relative to the current position/orientation
			 * of the robot - turn = 0; dx = 0; dy = 0; for no-op command.
			 */
			struct Command{
				/// Clockwise rotation in radians, range from -pi to pi
				double turn;
				/**
				 * offsets in m.
				 * 	x is always forwards
				 * 	y means crab walk (unimplemented in drivers as of now)
				 */
				double dx, dy;
			};
			
			/**
			 * It moves.
			 * 
			 * Will calculate commands necessary to implement a
			 * straight-line path to a given state, reacting to a
			 * continuously changing robot state.
			 * 
			 * Supports:
			 *  - Live updating of destination (desired state)
			 *  - Automatically compensates for errors, outside factors
			 * 		and incorrect actuation of the commands it outputs.
			 *  - Configurable robot movement speed
			 *  - Configurable threshold at which to consider a given
			 * 		state satisfied
			 */
			class Mover{
				private:
				State destination;
				State current_state;
				
				double stop_threshold;
				double forward_move_speed;
				public:
				Mover();
				Mover(const State& current_state,
					const State& destination,
					double forward_move_speed,
					double stop_threshold = 0.1);
				
				void setDestination(const State& new_destination);
				
				void setCurrentState(const State& state);
				
				void setMoveSpeed(double move_speed);
				
				void setStopThreshold(double new_stop_threshold);
				
				double getCorrectionAngleToDestination() const;
				/**
				 * Compute a command to move from the current state
				 * toward the desired state.
				 * 
				 * Note: angles are not enforced at the moment, only position.
				 */
				Command getCommand() const;
				
				/**
				 * Are we there yet?
				 */
				bool atDestination() const;
			};
		}
	}
}
