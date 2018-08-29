# wasp-des-rcv

Complimentary PROMELA and Stateflow models for the article named 'Design and Formal Verification of a Safe Stop Supervisor for an Automated Vehicle' by Jonas Krook, Lars Svensson, Yuchao Li, Lei Feng, and Martin Fabian.

## Spin

The Spin folder contains a file called [SupervisoryPathController.pml](Spin/SupervisoryPathController.pml), which contains one separate proctype for every major node in the system architecture. Additionally, LTL specifications are included in the bottom of that file. There are also Matlab code for running all verifications and printing the results to a LaTeX formatted table. Use [RunScript.m](Spin/RunScript.m) as an entry point to run all LTL specifications.

Spin needs to be installed and added to the path. Matlab 2017a was used to create the Matlab code, but older versions should be compatible.

## Simulink

The Simulink folder contains a [Simulink model](Stateflow/SupervisorStateMachine.mdl) in which there is a stateflow implementation of the supervisor. The subfolder [enums](Stateflow/enums) contains custom enumeration definitions which are required to be on the Matlab path if the Simulink model is to be run or if C++ code is to be generated.

The Simulink model was created with Matlab 2017a.
