#include <catch.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include <MoveStraightLine.hpp>

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
	SECTION("Direction, check correctness"){
		SECTION("Correct left"){
			State state;
			state.position = {0,0};
			state.direction = direction_vector_from_north(arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(-arma::datum::pi/6));
		}
		SECTION("Simultaneous heading + direction right"){
			State state;
			state.position = {0,-4};
			state.direction = direction_vector_from_north(arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(arma::datum::pi/4 - arma::datum::pi/6));
		}
		SECTION("Simultaneous heading + direction left"){
			State state;
			state.position = {0,4};
			state.direction = direction_vector_from_north(-arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(-arma::datum::pi/4 + arma::datum::pi/6));
		}
		SECTION("Simultaneous heading + direction left (opposite)"){
			State state;
			state.position = {0,4};
			state.direction = direction_vector_from_north(arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(-arma::datum::pi/4 - arma::datum::pi/6));
		}
		SECTION("Simultaneous heading + direction right (opposite)"){
			State state;
			state.position = {0,-4};
			state.direction = direction_vector_from_north(-arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(arma::datum::pi/4 + arma::datum::pi/6));
		}
		SECTION("Correct right"){
			State state;
			state.position = {0,0};
			state.direction = direction_vector_from_north(-arma::datum::pi/6);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(arma::datum::pi/6));
		}
		SECTION("Down and to the right"){
			State state;
			state.position = {5,-1};
			state.direction = direction_vector_from_north(5/4 * arma::datum::pi);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(-arma::datum::pi/4));
		}
		SECTION("Down and to the left, no actual correction needed"){
			State state;
			state.position = {5,1};
			state.direction = direction_vector_from_north(arma::datum::pi * 5/4);
			CAPTURE(state.direction);
			mover.setCurrentState(state);

			CHECK(mover.getCorrectionAngleToDestination() == Approx(0));
		}
	}
	// With the possibility of stopping to execute sharp turns
	SECTION("Direction, avoid sharp turns"){
		mover.setExplicitTurnThreshold(0.1);
		REQUIRE(mover.getExplicitTurnThreshold() == 0.1);
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
			state.direction = direction_vector_from_north(0.05);
			mover.setCurrentState(state);

			Command cmd = mover.getCommand();
			CHECK(cmd.dx == 1);
		}
		SECTION("Stop on > 0.2"){
			State state;
			state.position = {0,-4};
			// heading off by 0.2
			state.direction = direction_vector_from_north(arma::datum::pi/4 + 0.2);
			mover.setCurrentState(state);

			Command cmd = mover.getCommand();
			CHECK(cmd.dx == 0);
			CHECK(mover.getCorrectionAngleToDestination() == Approx(-0.2));
		}
		SECTION("Stop on > |-0.2|"){
			State state;
			state.position = {0,-4};
			// heading off by -0.2
			state.direction = direction_vector_from_north(arma::datum::pi/4 - 0.2);
			mover.setCurrentState(state);

			Command cmd = mover.getCommand();
			CHECK(cmd.dx == 0);
			CHECK(mover.getCorrectionAngleToDestination() == Approx(0.2));
		}

		SECTION("Go when < 0.2"){
			State state;
			state.position = {0,-4};
			// heading off by 0.2
			state.direction = direction_vector_from_north(arma::datum::pi/4 - 0.05);
			mover.setCurrentState(state);

			Command cmd = mover.getCommand();
			CHECK(cmd.dx == 1);
		}
	}
}

TEST_CASE("angle_from_north"){
	CHECK(angle_from_north(arma::vec{1,0}) == 0);
	CHECK(angle_from_north(arma::vec{0,1}) == arma::datum::pi/2);
	CHECK(angle_from_north(arma::vec{0,-1}) == arma::datum::pi * 3/2);
	CHECK(angle_from_north(arma::vec{-1,0}) == arma::datum::pi);
	CHECK(angle_from_north(arma::vec{4,-4}) == Approx(arma::datum::pi * 7/4));
	CHECK(angle_from_north(arma::vec{1,1}) == Approx(arma::datum::pi/4));
}

TEST_CASE("normalise_turning_angle"){
	CHECK(normalise_turning_angle(arma::datum::pi*2) == Approx(0));
	CHECK(normalise_turning_angle(0) == Approx(0));
	CHECK(normalise_turning_angle(arma::datum::pi) == Approx(arma::datum::pi));
	CHECK(normalise_turning_angle(-2*arma::datum::pi) == Approx(0));
	CHECK(normalise_turning_angle(-arma::datum::pi) == Approx(-arma::datum::pi));
	CHECK(normalise_turning_angle(-arma::datum::pi*3/2) == Approx(arma::datum::pi/2));

	CHECK(normalise_turning_angle(arma::datum::pi + arma::datum::pi/6) == Approx(-arma::datum::pi*5/6));
	CHECK(normalise_turning_angle(-arma::datum::pi - arma::datum::pi/6) == Approx(arma::datum::pi*5/6));
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

	expected = normalise(arma::vec{-0.5,-0.5});
	actual = direction_vector_from_north(arma::datum::pi * 5/4);
	CAPTURE(actual);
	CAPTURE(expected);
	CHECK(vec_close_enough(actual, expected, 0.001));
}
