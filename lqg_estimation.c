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

#include "linearizer/system_linearizer.h"
#include "kalman/kalman_filters.h"
#include "lqr/ilq_regulator.h"

#include <math.h>
#include <string.h>

#define DOFS_NUMBER_MAX 6

enum ControlState controlState = CONTROL_PASSIVE;

const char* DOF_NAMES[ DOFS_NUMBER_MAX ] = { "angle1", "angle2", "angle3", "angle4", "angle5", "angle6" };

double samplingTime = 0.0;

double positionProportionalGain = 0.0;
double forceProportionalGain = 0.0, forceIntegralGain = 0.0;

typedef struct DoFData
{
  LinearSystem linearSystem;
  KFilter observer;
  ILQRegulator regulator;

  double inputsList[ 1 ];
  double measuresList[ 1 ];
  double statesList[ 3 ];
  double impedancesList[ 3 ];
  double impedancesMinList[ 3 ];
  double feedbacksList[ 1 ];

  double totalForceSetpoint;
  double velocitySetpoint;
  double lastForceError;
}
DoFData;

DoFData dofsList[ DOFS_NUMBER_MAX ];
size_t dofsNumber = 0;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString )
{
  dofsNumber = strtoul( strtok( (char*) configurationString, " " ), NULL, 0 );
  if( dofsNumber > DOFS_NUMBER_MAX ) dofsNumber = DOFS_NUMBER_MAX;
  positionProportionalGain = strtod( strtok( NULL, " " ), NULL );
  forceProportionalGain = strtod( strtok( NULL, " " ), NULL );
  forceIntegralGain = strtod( strtok( NULL, " " ), NULL );
  
  for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
  {
    DoFData* dof = &(dofsList[ dofIndex ]);
    
    dof->linearSystem = SystemLinearizer_CreateSystem( 3, 1, LINEARIZATION_MAX_SAMPLES );
    dof->observer = Kalman_CreateFilter( 3, 1, 1 );
    dof->regulator = ILQR_Create( 3, 1, 0.00001 );
    
    Kalman_SetTransitionFactor( dof->observer, 2, 2, 0.0 );
    ILQR_SetTransitionFactor( dof->regulator, 2, 2, 0.0 );
  }
  
  return true;
}

void EndController() 
{ 
  for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
  {
    DoFData* dof = &(dofsList[ dofIndex ]);
    
    SystemLinearizer_DeleteSystem( dof->linearSystem );
    Kalman_DiscardFilter( dof->observer );
    ILQR_Delete( dof->regulator );
  }
  
  return; 
}

size_t GetJointsNumber() { return dofsNumber; }

const char** GetJointNamesList() { return DOF_NAMES; }

size_t GetAxesNumber() { return dofsNumber; }

const char** GetAxisNamesList() { return DOF_NAMES; }

size_t GetExtraInputsNumber( void ) { return 0; }
      
void SetExtraInputsList( double* inputsList ) { return; }

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { return; }

void SetControlState( enum ControlState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
  {
    DoFData* dof = &(dofsList[ dofIndex ]);
  
    if( controlState == CONTROL_CALIBRATION )
    {
      dof->impedancesMinList[ 0 ] = 0.0;
      dof->impedancesMinList[ 1 ] = 0.0;
      dof->impedancesMinList[ 2 ] = 0.0;
    }
    
    dof->velocitySetpoint = 0.0;
    
    Kalman_Reset( dof->observer );
  }
  
  controlState = newControlState;
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double deltaTime )
{
  samplingTime += deltaTime;
  
  for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
  {
    DoFData* dof = &(dofsList[ dofIndex ]);
  
    axisMeasuresList[ dofIndex ]->position = jointMeasuresList[ dofIndex ]->position;
    axisMeasuresList[ dofIndex ]->velocity = jointMeasuresList[ dofIndex ]->velocity;
    axisMeasuresList[ dofIndex ]->acceleration = jointMeasuresList[ dofIndex ]->acceleration;
    axisMeasuresList[ dofIndex ]->force = jointMeasuresList[ dofIndex ]->force;
    axisMeasuresList[ dofIndex ]->stiffness = jointMeasuresList[ dofIndex ]->stiffness;
    axisMeasuresList[ dofIndex ]->damping = jointMeasuresList[ dofIndex ]->damping;
    axisMeasuresList[ dofIndex ]->inertia = jointMeasuresList[ dofIndex ]->inertia;
    
    Kalman_SetTransitionFactor( dof->observer, 0, 1, deltaTime );
    Kalman_SetTransitionFactor( dof->observer, 0, 2, deltaTime * deltaTime / 2 );
    Kalman_SetTransitionFactor( dof->observer, 1, 2, deltaTime );
    ILQR_SetTransitionFactor( dof->regulator, 0, 1, deltaTime );
    ILQR_SetTransitionFactor( dof->regulator, 0, 2, deltaTime * deltaTime / 2 );
    ILQR_SetTransitionFactor( dof->regulator, 1, 2, deltaTime );
    
    if( controlState == CONTROL_OPERATION || controlState == CONTROL_CALIBRATION )
    {
      if( controlState == CONTROL_CALIBRATION ) axisSetpointsList[ 0 ]->position = sin( samplingTime ) / 2;
      
      double positionError = axisSetpointsList[ dofIndex ]->position - axisMeasuresList[ dofIndex ]->position;
      dof->statesList[ 0 ] = positionError;
      dof->statesList[ 1 ] = axisMeasuresList[ dofIndex ]->velocity;
      dof->statesList[ 2 ] = axisMeasuresList[ dofIndex ]->acceleration;
      dof->inputsList[ 0 ] = dof->totalForceSetpoint;
      SystemLinearizer_AddSample( dof->linearSystem, dof->statesList, dof->inputsList );
      SystemLinearizer_Identify( dof->linearSystem, dof->impedancesList );
       
      if( controlState == CONTROL_CALIBRATION ) 
      {       
        dof->impedancesList[ 0 ] = fmax( 0.0, dof->impedancesList[ 0 ] );
        dof->impedancesList[ 1 ] = fmax( 0.0, dof->impedancesList[ 1 ] );
        dof->impedancesList[ 2 ] = fmax( 0.0, dof->impedancesList[ 2 ] );
        dof->impedancesMinList[ 0 ] = ( dof->impedancesList[ 0 ] + dof->impedancesMinList[ 0 ] ) / 2;
        dof->impedancesMinList[ 1 ] = ( dof->impedancesList[ 1 ] + dof->impedancesMinList[ 1 ] ) / 2;
        dof->impedancesMinList[ 2 ] = ( dof->impedancesList[ 2 ] + dof->impedancesMinList[ 2 ] ) / 2;
        
        dof->totalForceSetpoint = positionProportionalGain * positionError;
      }
      else
      {
        dof->impedancesList[ 2 ] = fmax( dof->impedancesList[ 2 ], dof->impedancesMinList[ 2 ] );
        dof->impedancesList[ 1 ] = fmax( dof->impedancesList[ 1 ], dof->impedancesMinList[ 1 ] );
        dof->impedancesList[ 0 ] = fmax( dof->impedancesList[ 0 ], dof->impedancesMinList[ 0 ] );
        Kalman_SetTransitionFactor( dof->observer, 2, 1, -dof->impedancesList[ 1 ] / dof->impedancesList[ 2 ] );
        ILQR_SetTransitionFactor( dof->regulator, 2, 1, -dof->impedancesList[ 1 ] / dof->impedancesList[ 2 ] );
        Kalman_SetInputFactor( dof->observer, 2, 0, 1.0 / dof->impedancesList[ 2 ] );
        ILQR_SetInputFactor( dof->regulator, 2, 0, 1.0 / dof->impedancesList[ 2 ] );
        
        axisMeasuresList[ dofIndex ]->force = dof->impedancesList[ 0 ] * positionError;
        dof->inputsList[ 0 ] = axisMeasuresList[ dofIndex ]->force + dof->totalForceSetpoint;
        Kalman_Predict( dof->observer, dof->inputsList, dof->statesList );
        dof->measuresList[ 0 ] = -positionError;
        Kalman_Update( dof->observer, dof->measuresList, dof->statesList );
        ILQR_CalculateFeedback( dof->regulator, dof->statesList, dof->feedbacksList );
        
        dof->totalForceSetpoint = 0.0;//-dof->feedbacksList[ 0 ] + axisSetpointsList[ 0 ]->force;
      }
      
      double forceError = dof->totalForceSetpoint - axisMeasuresList[ 0 ]->force;
      dof->velocitySetpoint += forceProportionalGain * ( forceError - dof->lastForceError ) + forceIntegralGain * deltaTime * forceError;
      dof->lastForceError = forceError;
      
      axisMeasuresList[ dofIndex ]->stiffness = dof->impedancesList[ 0 ];
      axisMeasuresList[ dofIndex ]->damping = dof->impedancesList[ 1 ];
      axisMeasuresList[ dofIndex ]->inertia = dof->impedancesList[ 2 ];
    }
    
    axisSetpointsList[ dofIndex ]->velocity = dof->velocitySetpoint;
    
    jointSetpointsList[ dofIndex ]->position = axisSetpointsList[ dofIndex ]->position;
    jointSetpointsList[ dofIndex ]->velocity = axisSetpointsList[ dofIndex ]->velocity;
    jointSetpointsList[ dofIndex ]->acceleration = axisSetpointsList[ dofIndex ]->acceleration;
    jointSetpointsList[ dofIndex ]->force = dof->totalForceSetpoint;
  }
  
  fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, i=%.3f, d=%.3f, s=%.3f, vd=%.3f\n", axisSetpointsList[ 0 ]->position,                                    axisMeasuresList[ 0 ]->position,
                                                                                            axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                            axisMeasuresList[ 0 ]->inertia, axisMeasuresList[ 0 ]->damping, 
                                                                                            axisMeasuresList[ 0 ]->stiffness, axisSetpointsList[ 0 ]->velocity );
}
