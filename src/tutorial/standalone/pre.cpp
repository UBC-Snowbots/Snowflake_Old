#include <iostream>
#include <cstdlib>

int main(){
	for(int i = 0; i < 101; ++i){
		bool fizzbuzz = false;
		if(i % 3 == 0){
			fizzbuzz = true;
			std::cout << "fizz";
		}
		if(i % 5 == 0){
			fizzbuzz = true;
			std::cout << "buzz";
		}
		if(!fizzbuzz){
			std::cout << i;
		}
		std::cout << '\n';
	}
	return EXIT_SUCCESS;
}

