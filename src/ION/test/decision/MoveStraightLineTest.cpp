#include <catch.hpp>
#include <iostream>
#include <sstream>
#include <ION/decision/move_straight_line/MoveStraightLine.hpp>

using namespace ION::decision::move_straight_line;

const double MOVE_SPEED = 1;

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

TEST_CASE("Mover"){
	State initState;
	initState.position = {0,0};
	initState.direction = {1,0}; // straight up
	State desiredState;
	desiredState.position = {4,0};
	desiredState.direction = {1,0};
	Mover mover(
		initState,
		desiredState,
		MOVE_SPEED,
		0.1);
	
	SECTION("Already there"){
		State state;
		state.position = {4,0};
		state.direction = {1,0};
		mover.setCurrentState(state);
		
		Command cmd = mover.getCommand();
		CHECK(cmd == (Command{0, 0, 0}));
	}
	SECTION("Go forward"){
		Command cmd = mover.getCommand();
		Command expected{0, MOVE_SPEED, 0};
		CHECK(cmd == expected);
	}
	SECTION("Go left"){
		State state;
		state.position = {4,2};
		state.direction = {1,0};
		mover.setCurrentState(state);
		
		Command cmd = mover.getCommand();
		Command expected{-arma::datum::pi/2, MOVE_SPEED, 0};
		CHECK(cmd == expected);
	}
	SECTION("Go right"){
		State state;
		state.position = {4,-2};
		state.direction = {1,0};
		mover.setCurrentState(state);
		
		Command cmd = mover.getCommand();
		Command expected{arma::datum::pi/2, MOVE_SPEED, 0};
		CHECK(cmd == expected);
	}
	SECTION("Go back"){
		State state;
		state.position = {6,0};
		state.direction = {1,0};
		mover.setCurrentState(state);
		
		Command cmd = mover.getCommand();
		Command expected{arma::datum::pi, MOVE_SPEED, 0};
		CHECK(cmd == expected);
	}
}
