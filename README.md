# stepper_motor

## INSTALLATION
Clone this repository into your projects folder

# Add these lines to the CMakeLists.txt of your project
```sh
add_subdirectory(src\motor)
target_include_directories(main PUBLIC motor)
target_link_directories(main PRIVATE motor)
```
# include the library in your main.c program file eg.
```c
include 
```
