#!/bin/bash


function exit_script() {
    echo -e "\n\n***** error *****"
    exit 1
}

## determine if cmake exists 
which cmake > /dev/null
if [ $? -ne 0 ]; then
    echo "cmake not found"
    exit_script
fi


## create build directory
dir_name="build_linux"
rm -rf $dir_name

mkdir $dir_name
echo "create $dir_name"
cd $dir_name


## cmake
cmake -DCMAKE_BUILD_TYPE=Release ..
if [ $? -ne 0 ]; then
    echo -e "\n\n***** cmake failed *****\n\n"
    exit_script
else
    echo "cmake success"
fi

## make
make 
if [ $? -ne 0 ]; then
    echo -e "\n\n***** make failed *****\n\n"
    exit_script
else
    echo "make success"
fi


echo "end of file"
