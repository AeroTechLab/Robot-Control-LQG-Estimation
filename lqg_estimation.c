/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//  Copyright (c) 2019 Leonardo Consoni <consoni_2519@hotmail.com>             //
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

#include "linearizer/system_linearizer.h"
#include "kalman/kalman_filters.h"
#include "lqr/ilq_regulator.h"

#include <math.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

LinearSystem linearSystem = NULL;
KFilter observer = NULL;
ILQRegulator regulator = NULL;

double inputsList[ DOFS_NUMBER ];
double measuresList[ DOFS_NUMBER ];
double statesList[ DOFS_NUMBER * 3 ];
double impedancesList[ DOFS_NUMBER * 3 ];
double feedbacksList[ DOFS_NUMBER ];

double samplingTime = 0.0;

enum ControlState controlState = CONTROL_PASSIVE;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString )
{
  linearSystem = SystemLinearizer_CreateSystem( 3, 1, 500 );
  observer = Kalman_CreateFilter( 3, 1, 1 );
  regulator = ILQR_Create( 3, 1, 0.00001 );
  
  return true;
}

void EndController() 
{ 
  SystemLinearizer_DeleteSystem( linearSystem );
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
  
  controlState = newControlState;
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double elapsedTime )
{
  samplingTime += elapsedTime;
  
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  
  inputsList[ 0 ] = axisMeasuresList[ 0 ]->force;
  Kalman_Predict( observer, inputsList, statesList );
  measuresList[ 0 ] = axisMeasuresList[ 0 ]->position - axisSetpointsList[ 0 ]->position;
  Kalman_Update( observer, measuresList, statesList );
  ILQR_CalculateFeedback( regulator, statesList, feedbacksList );
  statesList[ 1 ] = axisMeasuresList[ 0 ]->velocity;
  statesList[ 2 ] = axisMeasuresList[ 0 ]->acceleration;
  SystemLinearizer_AddSample( linearSystem, statesList, inputsList );
  if( SystemLinearizer_Identify( linearSystem, impedancesList ) )
  {
    Kalman_SetTransitionFactor( observer, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
    ILQR_SetTransitionFactor( regulator, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
    Kalman_SetInputFactor( observer, 2, 0, 1.0 / impedancesList[ 2 ] );
    ILQR_SetInputFactor( regulator, 2, 0, 1.0 / impedancesList[ 2 ] );
  }
  axisMeasuresList[ 0 ]->stiffness = impedancesList[ 0 ];
  axisMeasuresList[ 0 ]->damping = impedancesList[ 1 ];
  axisMeasuresList[ 0 ]->inertia = impedancesList[ 2 ];
  if( controlState == CONTROL_OPERATION ) axisSetpointsList[ 0 ]->force = feedbacksList[ 0 ];
  else if( controlState == CONTROL_PREPROCESSING ) axisSetpointsList[ 0 ]->force = 10.0 * ( sin( elapsedTime ) - axisMeasuresList[ 0 ]->position );
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = axisSetpointsList[ 0 ]->force;
  jointSetpointsList[ 0 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  jointSetpointsList[ 0 ]->damping = axisSetpointsList[ 0 ]->damping;
}
