#include <iostream>
#include <cstdlib>
#include <vector>

bool is_prime(unsigned int n){
	if(n == 1) return false;
	static std::vector<unsigned int> prev;
	unsigned int start = prev.size() > 0 ? prev.back() : 2;
	for(unsigned int i = start; i < n; ++i){
		bool divides_by_any = false;
		for(std::vector<unsigned int>::iterator it = prev.begin(); it != prev.end(); ++it){
			if(i % *it == 0){
				divides_by_any = true;
				break;
			}
		}
		if(!divides_by_any){
			prev.push_back(i);
		}
	}
	for(std::vector<unsigned int>::iterator it = prev.begin(); it != prev.end(); ++it){
		if(n % *it == 0){
			return false;
		}
	}
	return true;
}

int main(){
	for(int i = 1; i < 101; ++i){
		if(is_prime(i)){
			std::cout << "prime";
		}else{
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
		}
		std::cout << '\n';
	}
	return EXIT_SUCCESS;
}

