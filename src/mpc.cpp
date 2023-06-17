/*
BSD 3-Clause License


Copyright (c) 2023, Mohamed Abdelkader Zahana

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <trajectory_generation/mpc.hpp>

MPC::MPC(){ return;}

MPC::~MPC(){return;}

void 
MPC::setQ(void)
{
   return;
}

void 
MPC::setQ6DoF(void)
{
   return;
}

void 
MPC::setR(void)
{
   return;
}

void 
MPC::setTransitionMatrix(void)
{
   return;
}

void 
MPC::setTransitionMatrix6DoF(void)
{
   return;
}

void 
MPC::setInputMatrix(void)
{
   return;
}

void 
MPC::setInputMatrix6DoF(void)
{
   return;
}

void 
MPC::setStateBounds(void)
{
   return;
}

void 
MPC::setStateBounds6DoF(void)
{
   return;
}


void 
MPC::setControlBounds(void)
{
   return;
}


void 
MPC::castMPCToQPHessian(void)
{
   return;
}

void 
MPC::castMPCToQPGradient(void)
{
   return;
}


void 
MPC::updateQPGradientVector(void)
{
   return;
}


void 
MPC::castMPCToQPConstraintMatrix(void)
{
   return;
}


void 
MPC::castMPCToQPConstraintBounds(void)
{
   return;
}

void 
MPC::updateQPConstraintsBounds(void)
{
   return;
}


bool 
MPC::initQPSolver(void)
{
   return true;
}


bool 
MPC::initMPCProblem(void)
{
   return true;
}


bool 
MPC::updateQP(void)
{
   return true;
}

bool 
MPC::mpcLoop(void)
{
   return true;
}


void 
MPC::extractSolution(void)
{
   return;
}

void 
MPC::extractSolution6Dof(void)
{
   return;
}

void 
MPC::printProblemInfo(void)
{
   return;
}


void 
MPC::testCases(void)
{
   return;
}


void 
MPC::saveMPCDataToFile(void)
{
   return;
}


void 
MPC::plotSolutions(void)
{
   return;
}