---
title: 使用boost::program_options解析命令行选项
date: 2021-10-21 12:07:32
tags:
- c++
categories:
- note
encrypt:
description:
---



Boost.ProgramOptions是Boost中一个专门用来解析命令行的库，其目标是轻松的解析命令行选项。



#### 基本用法

```c++
#include <boost/program_options.hpp>
#include <iostream>

void on_notifier(int a)
{
    std::cout << "On notifier: " << a << std::endl;
}

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("enable_something,e", "Enable Something")
        ("test_string,s", po::value<std::string>(), "Test String")
        ("test_int,p", po::value<int>(), "Test Int")
        ("test_default_value,d",po::value<int>()->default_value(1), "Test Default Value")
        ("test_notifier", value<int>()->notifier(on_notifier), "Test Notifier");
    po::variables_map vm;
  	po::store(po::parse_command_line(argc, argv, desc), vm);
  	po::notify(vm);
    
    // no options set
    // print all options
    if (argc == 1 || !vm.count("sensor_id"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    
    bool enable_something = vm.count("enable_something");
    int value = vm["test_default_value"].as<int>();
    if (vm.count("test_string"))
    {
        std::string ss = vm["test_string"].as<std::string>();
        std::cout << "test_string was set to: "  << ss << std::endl;
    }
    if (vm.count("test_int"))
    {
        int dd = vm["test_int"].as<int>();
        std::cout << "test_int was set to: "  << dd << std::endl;
    }

    return 0;
}
```



#### CMake里引入依赖

```cmake
find_package(Boost COMPONENTS program_options REQUIRED)
target_include_directories(myTarget PRIVATE ${Boost_INCLUDE_DIR})
target_link_libraries(myTarget ${Boost_LIBRARIES} )
```



[官方Tutorial参考][https://www.boost.org/doc/libs/1_63_0/doc/html/program_options/tutorial.html#idp523371328]

