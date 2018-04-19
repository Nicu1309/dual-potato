#!/bin/bash
DEP_FILE="rv_dependences.txt"
PROJECT_FOLDER=""
ENV_RISCV=""
USE_CUSTOM_GCC=""
CUSTOM_GCC=""
CUSTOM_GXX=""
YES=""
BUILD_TOOLS_COMMAND="CC=$CUSTOM_GCC CXX=$CUSTOM_GXX ./build.sh"
INSTALL_TOOLS=""

function usage {
    echo 'sudo <script> -r <RISCV_TOP> -p <DESTINATION_PROJECT> [-g <GCC_CUSTOM> -x <GXX_CUSTOM>] [-y] [-i]'
    echo ' -r RISCV installation path, where riscv-tools folder are OR will be placed and installed if [-i]'
    echo ' -i Install riscv-tools under RISCV_TOP directory'
    echo ' -p Destination folder for creating|initializing project'
    echo ' -g Custom GCC compiler. If this option is specified, -x must be also used'
    echo ' -x Custom GXX compiler. If this option is specified, -g must be also used'
    echo ' -y "Yes" as default answer. No user input needed'
    echo 'You will need verilator >= 3.920, available from https://www.veripool.org/projects/verilator/wiki/Installing'
}

function check_directory {
    if [ ! -d $1 ]; then
       echo "!!!!! $1 does not exist  !!!!!"
       if [ ! -n "$YES" ]; then
           echo "Do you want to create it? [y/n]"
           read create
           if [[ "$create" =~ 'n' ]];then
               echo "Then execute this command again, and specify an existent directory" 
               echo "Exiting now ..."
               exit 0
           fi
       fi
       echo "Creating directory $1"
       mkdir -p $1
    fi
}

# Check verilator version
function check_verilator_version {
    verilator_v=`verilator --version | cut -d' ' -f2`
    verilator_major=`echo $verilator_v | cut -d'.' -f1`
    verilator_minor=`echo $verilator_v | cut -d'.' -f2`
    if [[ $verilator_major -eq 3 && $verilator_minor -lt 900 || $verilator_major -lt 3 ]];then
       echo "verilator version not supported, please install verilator >= 3.9"
       exit 1
    fi
    echo "Checked verilator >= 3.9"
}

# Check gcc version
function check_gcc_version {
    gcc_v=`gcc --version | grep ^gcc | cut -d' ' -f4`
    gcc_major=`echo $gcc_v | cut -d'.' -f1`
    gcc_minor=`echo $gcc_v | cut -d'.' -f2`
    if [[ "$gcc_major" -eq 4 && "$gcc_minor" -lt 8 || "$gcc_major" -lt 4 ]];then
       echo "GCC version not supported, please install GCC >= 4.8"
       exit 1
    fi
    echo "Checked GCC >= 4.8"
}

# Check dependences for riscv-tools and ariane
function check_dependences {
    # Check that packages needed exist
    while read dep;do
        check=`dpkg -s $dep`
        if [ $? -gt 0 ];then
    	echo "Not installed $dep, installing dependency ..."
    	apt-get install $dep
        fi
    done < $DEP_FILE
}

function test_riscv_tools {
    pushd $ENV_RISCV 1>/dev/null
    export RISCV=$PWD
    popd 1>/dev/null
    export PATH=$PATH:/$RISCV/bin
    echo -e '#include <stdio.h>\n int main(void) { printf("Hello world!\\n"); return 0; }' > hello.c
    riscv64-unknown-elf-gcc -o hello hello.c
    if [[ "$?" -gt 0 ]]; then
        echo "Error while installing riscv-tools, please check that the directory [bin] is in the path. Try then executing the following: riscv64-unknown-elf-gcc -o hello hello.c"
        rm hello*
        exit 1
    fi
    rm hello*
}

function install_riscv_tools {
    # Clone and install riscv-tools
    check_directory $ENV_RISCV
    pushd $ENV_RISCV 1>/dev/null
    
    # Clone riscv-tools
    export RISCV=$PWD
    echo "Clonning riscv-tools into $RISCV"
    git clone https://github.com/riscv/riscv-tools.git 
    pushd "riscv-tools" 1>/dev/null
    git submodule update --init --recursive
    $BUILD_TOOLS_COMMAND
    echo "############### RISCV toolchain ##################" >> ~/.bashrc
    echo "export RISCV=$RISCV" >> ~/.bashrc
    echo "export PATH=$PATH:/$RISCV/bin" >> ~/.bashrc
    popd 1>/dev/null
    test_riscv_tools
    echo "========================================="
    echo "  RISCV toolchain correctly installed"
    echo "========================================="
    popd 1>/dev/null
}

function parse_options () {
    # Check if using non-default CC
    while getopts ":r:p:g:x:yi" myopt; do
        case $myopt in
          r)
              ENV_RISCV="$OPTARG"
              ;;
          i)
              INSTALL_TOOLS=1
              ;;
          p)
              PROJECT_FOLDER="$OPTARG"
              ;;
          g)
              CUSTOM_GCC="$OPTARG"
              USE_CUSTOM_GCC=1
              ;;
          x)
              CUSTOM_GXX="$OPTARG"
              ;;
	  y)
              YES=1
              ;;
         esac
    done
    # Check options destination folders are mandatory
    if [[ ! -n $ENV_RISCV || ! -n $PROJECT_FOLDER ]];then
       echo "!!!! Both RISCV and Project destination folders must be specified !!!!"
       usage
       exit
    fi 
}


###################################################
#                  MAIN
###################################################

# Check that we have admin rights
if [[ "$EUID" -gt 0 ]]; then
   echo "Run with sudo: sudo [script]"
   exit 1
fi

# Extract arguments
parse_options "$@"

# GCC version
# Check gcc version if we are not using custom gcc
if [[ ! -n "$USE_CUSTOM_GCC" ]];then
   echo "Checking gcc version"
   check_gcc_version
   BUILD_TOOLS_COMMAND="./build.sh"
fi

# Check verilator version, at least 3.9
check_verilator_version

# Check dependences for installing riscv-tools
echo "Checking dependences are installed"
check_dependences
echo "All dependences installed"

# If RISCV-TOOLS installation is specified, then install them under RISCV directory
if [[ -n $INSTALL_TOOLS ]]; then
    install_riscv_tools
else
    test_riscv_tools
fi

# Clone as submodule and build ariane project
echo "Adding ariane to project as submodule"
check_directory $PROJECT_FOLDER
pushd $PROJECT_FOLDER 1>/dev/null
[ -d .git ] || git init
[ -d ariane ] || git submodule add https://github.com/pulp-platform/ariane.git

# Clone as submodule the riscv-tests
echo "Adding tests to project as submodule"
[ -d riscv-tests ] || git submodule add https://github.com/riscv/riscv-tests.git

# Clone as submodule custom Pulp riscv-fesvr
echo "Adding custom Pulp riscv-fesvr to project as submodule"
[ -d riscv-fesvr ] || git submodule add https://github.com/pulp-platform/riscv-fesvr.git
git submodule update --init --recursive
pushd riscv-fesvr 1>/dev/null
mkdir build 
pushd build 1>/dev/null
../configure --prefix=$RISCV
make install
popd 1>/dev/null
# Check whether we have already installed this tools  
if [[ `grep -Fq "Ariane-fesvr" ~/.bashrc` ]];then
    echo "############### Ariane-fesvr ##############" >> ~/.bashrc
    echo "export LIBRARY_PATH=$LIBRARY_PATH:$RISCV/lib" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=$LIBRARY_PATH:$RISCV/lib" >> ~/.bashrc
    echo "export C_INCLUDE_PATH=$LIBRARY_PATH:$RISCV/include" >> ~/.bashrc
    echo "export CPLUS_INCLUDE_PATH=$LIBRARY_PATH:$RISCV/include" >> ~/.bashrc
fi
export LIBRARY_PATH=$LIBRARY_PATH:$RISCV/lib
export LD_LIBRARY_PATH=$LIBRARY_PATH:$RISCV/lib
export C_INCLUDE_PATH=$LIBRARY_PATH:$RISCV/include
export CPLUS_INCLUDE_PATH=$LIBRARY_PATH:$RISCV/include
popd 1>/dev/null

# Make verilate command
pushd ariane 1>/dev/null
make verilate
popd 1>/dev/null

# Build tests
pushd riscv-tests 1>/dev/null
autoconf
./configure --prefix=$RISCV/target
make
make install
popd 1>/dev/null
popd 1>/dev/null
