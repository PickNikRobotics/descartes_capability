#!/bin/bash -eu

# Desc:
#  A bash script for customizing this template for your fancy new package.
#
# Usage:
#  . ./customize_package.sh

# NOTE:
#  This must be run in the PACKAGE_NAME directory and should only be run once.

# I'm not sure why this is necessary but we re-source it:
source ~/bitchin_unix/scripts/search.sh

FOLDER_NAME=${PWD##*/}

echo ""
echo "PickNik Blanket - A Template for ROS Packages"
echo ""
echo "Package parameters -------------------------------------------------"
echo ""
read -p "Name your package. Should be the same as your github repo name for the README.md ($FOLDER_NAME): " PACKAGE_NAME
if [ "$PACKAGE_NAME" == "" ]; then
    PACKAGE_NAME="$FOLDER_NAME"
fi
read -p "Create a description for your package. This will be used in the README and package.xml: " PACKAGE_DESCRIPTION
if [ "$PACKAGE_DESCRIPTION" == "" ]; then
        PACKAGE_DESCRIPTION="The package description"
fi
read -p "Your first name (John): " FIRST_NAME
if [ "$FIRST_NAME" == "" ]; then
  echo "Field required"
  exit 1
fi
read -p "Your last name (Doe): " LAST_NAME
if [ "$LAST_NAME" == "" ]; then
  echo "Field required"
  exit 1
fi
read -p "Your email address (john@picknik.ai): " EMAIL_ADDRESS
if [ "$EMAIL_ADDRESS" == "" ]; then
  echo "Field required"
  exit 1
fi
read -p "Your github name (johnbot): " GITHUB_NAME
if [ "$GITHUB_NAME" == "" ]; then
  echo "Field required"
  exit 1
fi

###########################################################################
echo ""
echo "CPP Class Parameters -------------------------------------------------"
echo ""
read -p "Name your cpp object class for your library (ClassName): " CPP_CLASS_NAME
if [ "$CPP_CLASS_NAME" == "" ]; then
    CPP_CLASS_NAME="ClassName"
fi
read -p "Name your cpp class file name (class_name): " CPP_CLASS_FILE_NAME
if [ "$CPP_CLASS_FILE_NAME" == "" ]; then
    CPP_CLASS_FILE_NAME="class_name"
fi

# DTC: I think short name should be same as class name
CPP_SHORT_NAME=$CPP_CLASS_FILE_NAME

###########################################################################
echo ""
echo "CPP Executable Parmeters-----------------------------------------------"
echo ""
read -p "Name your cpp executable filename (${CPP_CLASS_FILE_NAME}_main): " CPP_EXECUTABLE_NAME
if [ "$CPP_EXECUTABLE_NAME" == "" ]; then
    CPP_EXECUTABLE_NAME="${CPP_CLASS_FILE_NAME}_main"
fi

###########################################################################
echo ""
echo "Python Class Parameters -------------------------------------------------"
echo ""
read -p "Name your python object class for your library (PythonClassName): " PYTHON_CLASS_NAME
if [ "$PYTHON_CLASS_NAME" == "" ]; then
    PYTHON_CLASS_NAME="PythonClassName"
fi
read -p "Name your python class file name (python_class_name): " PYTHON_CLASS_FILE_NAME
if [ "$PYTHON_CLASS_FILE_NAME" == "" ]; then
    PYTHON_CLASS_FILE_NAME="python_class_name"
fi


###########################################################################
echo ""
echo "Python Executable Parmeters-----------------------------------------------"
echo ""
read -p "Name your python executable filename (${PYTHON_CLASS_FILE_NAME}.py): " PYTHON_EXECUTABLE_NAME
if [ "$PYTHON_EXECUTABLE_NAME" == "" ]; then
    PYTHON_EXECUTABLE_NAME="${PYTHON_CLASS_FILE_NAME}.py"
fi

declare -a array_ans=(
        "$PACKAGE_NAME"
        "$PACKAGE_DESCRIPTION"
        "$CPP_CLASS_FILE_NAME"
        "$CPP_EXECUTABLE_NAME"
        "$CPP_CLASS_NAME"
        "$CPP_SHORT_NAME"
        "$PYTHON_CLASS_NAME"
        "$PYTHON_CLASS_FILE_NAME"
        "$PYTHON_EXECUTABLE_NAME"
        "$FIRST_NAME"
        "$LAST_NAME"
        "$EMAIL_ADDRESS"
        "$GITHUB_NAME"
)

declare -a array=(
        "PACKAGE_NAME"
        "PACKAGE_DESCRIPTION"
        "CPP_CLASS_FILE_NAME"
        "CPP_EXECUTABLE_NAME"
        "CPP_CLASS_NAME"
        "CPP_SHORT_NAME"
        "PYTHON_CLASS_NAME"
        "PYTHON_CLASS_FILE_NAME"
        "PYTHON_EXECUTABLE_NAME"
        "FIRST_NAME"
        "LAST_NAME"
        "EMAIL_ADDRESS"
        "GITHUB_NAME"
)


# Iterate over the replacements
set -x
for ((i=0; i < ${#array_ans[@]} ; i++ ))
do
    findreplacefilename "${array[i]}" "${array_ans[i]}"
    findreplacehidden "${array[i]}" "${array_ans[i]}"
done
set +x

echo ""
echo "Deleting temporary files..."
echo ""

# Delete temp files
rm customize_package.sh
rm SETUP_BLANKET.md

echo ""
echo "Done! -----------------------------------------------"
echo ""

# A reminder to fix the ifdef's
# TODO: automate this
read -p "Don't forget to go through the .h files and capitalize the ifdef's" dummy
