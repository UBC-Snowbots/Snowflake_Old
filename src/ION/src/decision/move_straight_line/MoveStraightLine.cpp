#include <ION/decision/move_straight_line/MoveStraightLine.hpp>

using namespace arma;

namespace ION{
	namespace decision{
		namespace move_straight_line{
			
			vec direction_vector(double angle){
				return vec{sin(angle), cos(angle)};
			}
			double angle_from_north(const vec& direction){
				const vec north = {1, 0};
				// calculate acute angle between north and direction
				// using dot product
				double acute_angle = acos(
					norm_dot(
						north, 
						direction));
				// horizontal component is Y in our coordinate space
				double horizontal_component = direction[1];
				
				return horizontal_component < 0 ?
					2*datum::pi - acute_angle : // if past 180 degrees,
							// correct angle to be obtuse, then we can
							// rely on signs being meaningful when
							// adding/subtracting
					acute_angle; // otherwise we really have an acute
							// angle
			}
			double normalise_turning_angle(double angle){
				return angle > datum::pi ?
					-(datum::pi*2 - angle) :
					angle;
			}
			
			Mover::Mover(const State& current_state,
				const State& destination,
				double forward_move_speed,
				double stop_threshold ):
					current_state(current_state),
					destination(destination),
					forward_move_speed(forward_move_speed),
					stop_threshold(stop_threshold)
				{}
			
			void Mover::setDestination(const State& new_destination){
				destination = new_destination;
			}
			
			void Mover::setCurrentState(const State& state){
				current_state = state;
			}
			
			void Mover::setMoveSpeed(double move_speed){
				forward_move_speed = move_speed;
			}
			
			void Mover::setStopThreshold(double new_stop_threshold){
				stop_threshold = new_stop_threshold;
			}
			
			double Mover::getCorrectionAngleToDestination() const{
				vec dest = destination.position;
				vec pos = current_state.position;
				// normalise to angle from north
				double angle_to_destination = angle_from_north(dest - pos);
				double current_angle = angle_from_north(current_state.direction);
				
				// return normalised angle so drivers don't turn obtuse angles
				return normalise_turning_angle(
					angle_to_destination - current_angle);
			}
			/**
			 * Compute a command to go from the current state to the desired state.
			 * Note: angles are not enforced, only position.
			 */
			Command Mover::getCommand() const{
				Command retCommand;
				
				// stop if at destination
				bool move = !atDestination();
				
				retCommand.dx = move ? forward_move_speed : 0;
				retCommand.dy = 0;
				
				retCommand.turn = move ? getCorrectionAngleToDestination() : 0;
				
				return retCommand;
			}
			
			bool Mover::atDestination() const{
				vec offset_from_dest = current_state.position - destination.position;
				double distance_remaining = sqrt(dot(offset_from_dest, offset_from_dest));
				
				return distance_remaining <= stop_threshold;
			}
		}
	}
}
