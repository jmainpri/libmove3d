#!/bin/bash

# Make A Directory Named "build_test"
# compile the flag COMPILE_MANIPULATION_PLANNER_TESTING
#

echo "Bash version ${BASH_VERSION}..."
echo "Testing the Manipulation planner: "

for i in {1..5}
 do
    for j in {0..10}
        do
            echo "---------------------------------------------"
            echo "Test suite number $i :"
            echo "---------------------------------------------"
            echo "---------------------------------------------"
            echo "Test number $j :"
            echo "---------------------------------------------"
            ../../build_test/Debug/bin/${HOSTTYPE}/move3d -f ../../../BioMove3DDemos/GS/gsJidoKukaSAHandSM.p3d -sc ../../../BioMove3DDemos/GS/SCENARIO/ManipulationTestSAHand.sce -c pqp -test $i
    done
done