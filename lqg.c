/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//  Copyright (c) 2019-2020 Leonardo Consoni <leonardojc@protonmail.com>       //
//                                                                             //
//  This file is part of Robot-Control-LQG-Estimation.                         //
//                                                                             //
//  Robot-Control-LQG-Estimation is free software: you can redistribute it     //
//  and/or modify it under the terms of the GNU Lesser General Public License  //
//  as published by the Free Software Foundation, either version 3 of the      //
//  License, or (at your option) any later version.                            //
//                                                                             //
//  Robot-Control-LQG-Estimation is distributed in the hope that it will       //
//  be useful, but WITHOUT ANY WARRANTY; without even the implied warranty     //
//  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            //
//  GNU Lesser General Public License for more details.                        //
//                                                                             //
//  You should have received a copy of the GNU Lesser General Public License   //
//  along with RobotSystem-Lite. If not, see <http://www.gnu.org/licenses/>.   //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////


#include "interface/robot_control.h"

#include "kalman/kalman_filters.h"
#include "lqr/ilq_regulator.h"

#include <math.h>
#include <string.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

KFilter observer = NULL;
ILQRegulator regulator = NULL;

double inputsList[ DOFS_NUMBER ] = { 0 };
double measuresList[ DOFS_NUMBER ] = { 0 };
double statesList[ DOFS_NUMBER * 3 ] = { 0 };
double impedancesList[ DOFS_NUMBER * 3 ] = { 0 };
double impedancesMinList[ DOFS_NUMBER * 3 ] = { 0 };
double feedbacksList[ DOFS_NUMBER ] = { 0 };

double samplingTime = 0.0;

enum ControlState controlState = CONTROL_PASSIVE;

double positionProportionalGain = 0.0;
double forceProportionalGain = 0.0, forceIntegralGain = 0.0;

double totalForceSetpoint = 0.0;
double velocitySetpoint = 0.0;
double lastForceError = 0.0;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString )
{
  positionProportionalGain = strtod( strtok( (char*) configurationString, " " ), NULL );
  forceProportionalGain = strtod( strtok( NULL, " " ), NULL );
  forceIntegralGain = strtod( strtok( NULL, " " ), NULL );
  
  observer = Kalman_CreateFilter( 3, 1, 1 );
  regulator = ILQR_Create( 3, 1, 0.00001 );
  
  Kalman_SetTransitionFactor( observer, 2, 2, 0.0 );
  ILQR_SetTransitionFactor( regulator, 2, 2, 0.0 );
  
  return true;
}

void EndController() 
{ 
  Kalman_DiscardFilter( observer );
  ILQR_Delete( regulator );
  
  return; 
}

size_t GetJointsNumber() { return DOFS_NUMBER; }

const char** GetJointNamesList() { return DOF_NAMES; }

size_t GetAxesNumber() { return DOFS_NUMBER; }

const char** GetAxisNamesList() { return DOF_NAMES; }

size_t GetExtraInputsNumber( void ) { return 0; }
      
void SetExtraInputsList( double* inputsList ) { return; }

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { return; }

void SetControlState( enum ControlState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  if( controlState == CONTROL_CALIBRATION )
  {
    impedancesMinList[ 0 ] = impedancesList[ 0 ];
    impedancesMinList[ 1 ] = impedancesList[ 1 ];
    impedancesMinList[ 2 ] = impedancesList[ 2 ];
  }
  
  controlState = newControlState;
  
  velocitySetpoint = 0.0;
  
  Kalman_Reset( observer );
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double deltaTime )
{
  samplingTime += deltaTime;
  
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  axisMeasuresList[ 0 ]->inertia = jointMeasuresList[ 0 ]->inertia;
  axisMeasuresList[ 0 ]->damping = jointMeasuresList[ 0 ]->damping;
  axisMeasuresList[ 0 ]->stiffness = jointMeasuresList[ 0 ]->stiffness;
  
  Kalman_SetTransitionFactor( observer, 0, 1, deltaTime );
  Kalman_SetTransitionFactor( observer, 0, 2, deltaTime * deltaTime / 2 );
  Kalman_SetTransitionFactor( observer, 1, 2, deltaTime );
  ILQR_SetTransitionFactor( regulator, 0, 1, deltaTime );
  ILQR_SetTransitionFactor( regulator, 0, 2, deltaTime * deltaTime / 2 );
  ILQR_SetTransitionFactor( regulator, 1, 2, deltaTime );
  
  if( controlState == CONTROL_OPERATION || controlState == CONTROL_CALIBRATION )
  {    
    inputsList[ 0 ] = axisMeasuresList[ 0 ]->force + totalForceSetpoint;
    measuresList[ 0 ] = axisMeasuresList[ 0 ]->position - axisSetpointsList[ 0 ]->position;
    
    if( controlState == CONTROL_CALIBRATION ) 
    {
      axisSetpointsList[ 0 ]->position = sin( samplingTime );
      
      impedancesList[ 2 ] = axisMeasuresList[ 0 ]->inertia;
      impedancesList[ 1 ] = axisMeasuresList[ 0 ]->damping;
      impedancesList[ 0 ] = axisMeasuresList[ 0 ]->stiffness;
      
      totalForceSetpoint = -positionProportionalGain * measuresList[ 0 ];
    }
    else
    {
      impedancesList[ 2 ] = fmax( axisMeasuresList[ 0 ]->inertia, impedancesMinList[ 2 ] );
      impedancesList[ 1 ] = fmax( axisMeasuresList[ 0 ]->damping, impedancesMinList[ 1 ] );
      impedancesList[ 0 ] = fmax( axisMeasuresList[ 0 ]->stiffness, impedancesMinList[ 0 ] );
      Kalman_SetTransitionFactor( observer, 2, 0, -impedancesList[ 0 ] / impedancesList[ 2 ] );
      ILQR_SetTransitionFactor( regulator, 2, 0, -impedancesList[ 0 ] / impedancesList[ 2 ] );
      Kalman_SetTransitionFactor( observer, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
      ILQR_SetTransitionFactor( regulator, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
      Kalman_SetInputFactor( observer, 2, 0, 1.0 / impedancesList[ 2 ] );
      ILQR_SetInputFactor( regulator, 2, 0, 1.0 / impedancesList[ 2 ] );
      
      Kalman_Predict( observer, inputsList, statesList );
      Kalman_Update( observer, measuresList, statesList );
      
      ILQR_CalculateFeedback( regulator, statesList, feedbacksList );
  
      totalForceSetpoint = 0.0;//-feedbacksList[ 0 ] + axisSetpointsList[ 0 ]->force;
    } 
    
    double forceError = totalForceSetpoint - axisMeasuresList[ 0 ]->force;
    velocitySetpoint += forceProportionalGain * ( forceError - lastForceError ) + forceIntegralGain * deltaTime * forceError;
    lastForceError = forceError;
  }
  
  axisSetpointsList[ 0 ]->velocity = velocitySetpoint;
  
  fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, i=%.3f, d=%.3f, s=%.3f, vd=%.3f\n", axisSetpointsList[ 0 ]->position, axisMeasuresList[ 0 ]->position,
                                                                                          axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                          axisMeasuresList[ 0 ]->inertia, axisMeasuresList[ 0 ]->damping, 
                                                                                          axisMeasuresList[ 0 ]->stiffness, axisSetpointsList[ 0 ]->velocity );
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = totalForceSetpoint;
}

