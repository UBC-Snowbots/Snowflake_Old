#include <catch.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <ION/decision/move_straight_line/MoveStraightLine.hpp>

using namespace ION::decision::move_straight_line;

bool operator==(const Command& a, const Command& b){
	return a.dx == b.dx
		&& a.dy == b.dy
		&& a.turn == b.turn;
}

/*std::ostream& operator<<(std::ostream& out, const Command& cmd){
	out << "{dx: " << cmd.dx << ", dy: " << cmd.dy << ", turn: " << cmd.turn << "}";
	return out;
}*/

std::ostream& operator<<(std::ostream& out, const Command& cmd){
	return out << "{dx: " << cmd.dx << ", dy: " << cmd.dy << ", turn: " << cmd.turn << "}";
}

std::string to_string(const Command& cmd){
	std::ostringstream out;
	out << cmd;
	return out.str();
}

/*namespace Catch {
    std::string toString(const Command& cmd) {
		std::ostringstream out;
		operator<<(out, &cmd);
		//std::cout << cmd << std::endl;
        return out.str();
    }
}*/

const double MOVE_SPEED = 1;

bool test_command_close_enough(const Command& a, const Command& b, double threshold){
	double diff = (abs(a.dx - b.dx) + abs(a.dy - b.dy) + abs(a.turn - b.turn)) / 3;
	return diff < threshold;
}

TEST_CASE("Mover"){
	Mover mover;{
		State initState;
		initState.position = {0,0};
		initState.direction = {1,0}; // straight up
		mover.setCurrentState(initState);
		
		State desiredState;
		desiredState.position = {4,0};
		desiredState.direction = {1,0};
		mover.setDestination(desiredState);
		
		mover.setMoveSpeed(MOVE_SPEED);
	}
	
	SECTION("Position"){
		mover.setExplicitTurnThreshold(arma::datum::pi*2);
		SECTION("Already there"){
			State state;
			state.position = {4,0};
			state.direction = {1,0};
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			CHECK(cmd == (Command{0, 0, 0}));
			CHECK(mover.atDestination());
		}
		SECTION("stop threshold"){
			CHECK(!mover.atDestination());
			mover.setStopThreshold(4);
			CHECK(mover.atDestination());
		}
		SECTION("Go forward"){
			Command cmd = mover.getCommand();
			Command expected{0, MOVE_SPEED, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
		SECTION("Go left"){
			State state;
			state.position = {4,2};
			state.direction = {1,0};
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			Command expected{-arma::datum::pi/2, MOVE_SPEED, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
		SECTION("Go right"){
			State state;
			state.position = {4,-2};
			state.direction = {1,0};
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			Command expected{arma::datum::pi/2, MOVE_SPEED, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
		SECTION("Go back"){
			State state;
			state.position = {6,0};
			state.direction = {1,0};
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			Command expected{arma::datum::pi, MOVE_SPEED, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
	}
	// With the possibility of stopping to execute sharp turns
	SECTION("Direction"){
		SECTION("Turn only"){
			State state;
			state.position = {6,0};
			state.direction = {1,0};
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			Command expected{arma::datum::pi, 0, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
		SECTION("Move only"){
			Command cmd = mover.getCommand();
			Command expected{0, MOVE_SPEED, 0};
			CHECK(cmd == expected);
			CHECK(!mover.atDestination());
		}
		SECTION("Move and turn"){
			State state;
			state.position = {0,0};
			// threshold defaults to pi/8, so we should still move
			state.direction = direction_vector_from_north(arma::datum::pi/16);
			mover.setCurrentState(state);
			
			Command cmd = mover.getCommand();
			Command expected{-arma::datum::pi/16, MOVE_SPEED, 0};
			INFO("cmd = " << to_string(cmd));
			INFO("expected = " << to_string(expected));
			CHECK(test_command_close_enough(cmd, expected, 0.00001));
		}
	}
}

TEST_CASE("angle_from_north"){
	CHECK(angle_from_north(arma::vec{1,0}) == 0);
	CHECK(angle_from_north(arma::vec{0,1}) == arma::datum::pi/2);
	CHECK(angle_from_north(arma::vec{0,-1}) == arma::datum::pi * 3/2);
	CHECK(angle_from_north(arma::vec{-1,0}) == arma::datum::pi);
}

bool vec_close_enough(arma::vec& a, arma::vec& b, double tolerance){
	return (arma::accu( arma::abs(a-b) ) / a.n_elem) < tolerance;
}

TEST_CASE("direction_angle"){
	arma::vec expected, actual;
	
	expected = {1,0};
	actual = direction_vector_from_north(0);
	CAPTURE(actual);
	CAPTURE(expected);
	CHECK(vec_close_enough(actual, expected, 0.001));
	
	expected = {-1,0};
	actual = direction_vector_from_north(arma::datum::pi);
	CAPTURE(actual);
	CAPTURE(expected);
	CHECK(vec_close_enough(actual, expected, 0.001));
	
	expected = {0,1};
	actual = direction_vector_from_north(arma::datum::pi/2);
	CAPTURE(actual);
	CAPTURE(expected);
	CHECK(vec_close_enough(actual, expected, 0.001));
	
	expected = {0,-1};
	actual = direction_vector_from_north(arma::datum::pi * 3/2);
	CAPTURE(actual);
	CAPTURE(expected);
	CHECK(vec_close_enough(actual, expected, 0.001));
}
