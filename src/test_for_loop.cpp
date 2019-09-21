#include <ros/ros.h>
#include <omp.h>

void Loop(size_t size, size_t skip)
{
	std::cout << "size = " << size << std::endl;
	std::cout << "skip = " << skip << std::endl;

	std::vector<int> list;
	/* std::vector<int> list1; */
	list.resize((size-1)/skip+1);
	int counter = 0;
	
	#pragma omp parallel for reduction(+:counter)
	for(size_t i=0;i<size;i+=skip){
		list[i/skip] = i;
		// list1.push_back(i);
		++counter;

		/* if(i==0)	list.erase(list.begin()); */
	}

	struct index_and_value{
		size_t index;
		size_t value;
	};
	std::vector<index_and_value> error_list;
	for(size_t i=0;i<list.size();++i){
		std::cout << "list[" << i << "] = " << list[i] << std::endl;
		if(list[i] != i*skip){
			index_and_value error;
			error.index = i;
			error.value = list[i];
			error_list.push_back(error);
		}
	}
	/* for(size_t i=0;i<list1.size();++i)	std::cout << "list1[" << i << "] = " << list1[i] << std::endl; */
	for(size_t i=0;i<error_list.size();++i)	std::cout << "error: list[" << error_list[i].index << "] = " << error_list[i].value << std::endl;

	std::cout << "counter = " << counter << std::endl;
	std::cout << "list.size() = " << list.size() << std::endl;
	if(counter != list.size())	std::cout << "error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_for_loop");
	
	/* Loop(9, 3); */
	Loop(100, 3);
}
