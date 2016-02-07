#include <MoveStraightLine.hpp>

using namespace arma;

namespace ION{
	namespace decision{
		namespace move_straight_line{

			vec direction_vector_from_north(double angle){
				return vec{cos(angle), sin(angle)};
			}
			double angle_from_north(vec direction){
				const vec north = {1, 0};
				if(all(direction == vec{0,0})){
					direction = {0.0000001, 0};
				}
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
				double limited_angle = remainder(angle, datum::pi*2);
				if(limited_angle > datum::pi){
					// too far to the right
					return -2*datum::pi + limited_angle;
				}else if(limited_angle < -datum::pi){
					// too far to the left
					return 2*datum::pi + limited_angle;
				}else{
					return limited_angle;
				}
			}

			Mover::Mover(){}

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

			void Mover::setExplicitTurnThreshold(double new_explicit_turn_threshold){
				explicit_turn_threshold = new_explicit_turn_threshold;
			}

			double Mover::getStopThreshold() const{
				return stop_threshold;
			}

			double Mover::getMoveSpeed() const{
				return forward_move_speed;
			}

			double Mover::getExplicitTurnThreshold() const{
				return explicit_turn_threshold;
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

				double correction_angle_to_destination = getCorrectionAngleToDestination();
				// should we stop and only turn until we're at least
				// sort of pointing in the right direction?
				bool explicit_turn = fabs(correction_angle_to_destination) > explicit_turn_threshold;

				retCommand.dx = move && !explicit_turn ? forward_move_speed : 0;
				retCommand.dy = 0;

				retCommand.turn = move ? correction_angle_to_destination : 0;

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
