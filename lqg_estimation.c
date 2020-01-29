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

bool identificationEnabled = false;

double positionProportionalGain = 0.0;
double interactionForceSetpoint = 0.0;

double inputsList[ 1 ] = { 0 };
double measuresList[ 1 ] = { 0 };
double statesList[ 3 ] = { 0 };
double impedancesList[ 3 ] = { 0 };
double feedbacksList[ 1 ] = { 0 };

typedef struct DoFData
{
  LinearSystem linearSystem;
  KFilter observer;
  ILQRegulator regulator;

  double impedancesMinList[ 3 ];
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
  identificationEnabled = ( strcmp( strtok( NULL, " " ), "id_on" ) == 0 ) ? true : false;
  
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
    
    double interactionForceSetpoint = 0.0;
    double actuatorForceSetpoint = 0.0;
    
    if( controlState == CONTROL_OPERATION || controlState == CONTROL_CALIBRATION )
    {
      if( controlState == CONTROL_CALIBRATION ) axisSetpointsList[ 0 ]->position = sin( samplingTime ) / 2;
      
      measuresList[ 0 ] = axisSetpointsList[ dofIndex ]->position - axisMeasuresList[ dofIndex ]->position;
      feedbacksList[ 0 ] = positionProportionalGain * measuresList[ 0 ];
      
      if( identificationEnabled )
      {
        statesList[ 0 ] = measuresList[ 0 ];
        statesList[ 1 ] = axisMeasuresList[ dofIndex ]->velocity;
        statesList[ 2 ] = axisMeasuresList[ dofIndex ]->acceleration;
        inputsList[ 0 ] = -axisMeasuresList[ 0 ]->force;
        SystemLinearizer_AddSample( dof->linearSystem, statesList, inputsList );
        SystemLinearizer_Identify( dof->linearSystem, impedancesList );
      
        if( controlState == CONTROL_CALIBRATION ) 
        {       
          impedancesList[ 0 ] = fmax( 0.0, impedancesList[ 0 ] );
          impedancesList[ 1 ] = fmax( 0.0, impedancesList[ 1 ] );
          impedancesList[ 2 ] = fmax( 0.0, impedancesList[ 2 ] );
          dof->impedancesMinList[ 0 ] = ( impedancesList[ 0 ] + dof->impedancesMinList[ 0 ] ) / 2;
          dof->impedancesMinList[ 1 ] = ( impedancesList[ 1 ] + dof->impedancesMinList[ 1 ] ) / 2;
          dof->impedancesMinList[ 2 ] = ( impedancesList[ 2 ] + dof->impedancesMinList[ 2 ] ) / 2;
        }
        else
        {
          impedancesList[ 0 ] = fmax( impedancesList[ 0 ], dof->impedancesMinList[ 0 ] );
          impedancesList[ 1 ] = fmax( impedancesList[ 1 ], dof->impedancesMinList[ 1 ] );
          impedancesList[ 2 ] = fmax( impedancesList[ 2 ], dof->impedancesMinList[ 2 ] );
          
          Kalman_SetTransitionFactor( dof->observer, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
          ILQR_SetTransitionFactor( dof->regulator, 2, 1, -impedancesList[ 1 ] / impedancesList[ 2 ] );
          Kalman_SetInputFactor( dof->observer, 2, 0, 1.0 / impedancesList[ 2 ] );
          ILQR_SetInputFactor( dof->regulator, 2, 0, 1.0 / impedancesList[ 2 ] );
          
          inputsList[ 0 ] = impedancesList[ 0 ] * measuresList[ 0 ] + interactionForceSetpoint;
          Kalman_Predict( dof->observer, inputsList, statesList );
          Kalman_Update( dof->observer, measuresList, statesList );
          ILQR_CalculateFeedback( dof->regulator, statesList, feedbacksList );
        }
      }
      
      interactionForceSetpoint = 0.0;//feedbacksList[ 0 ] + axisSetpointsList[ 0 ]->force;
      
      actuatorForceSetpoint = axisMeasuresList[ dofIndex ]->inertia * axisMeasuresList[ dofIndex ]->acceleration
                              + axisMeasuresList[ dofIndex ]->damping * axisMeasuresList[ dofIndex ]->velocity
                              + axisMeasuresList[ dofIndex ]->stiffness * axisMeasuresList[ dofIndex ]->position
                              + interactionForceSetpoint;
      
      axisMeasuresList[ dofIndex ]->stiffness = impedancesList[ 0 ];
      axisMeasuresList[ dofIndex ]->damping = impedancesList[ 1 ];
      axisMeasuresList[ dofIndex ]->inertia = impedancesList[ 2 ];
    }
    
    jointSetpointsList[ dofIndex ]->position = axisSetpointsList[ dofIndex ]->position;
    jointSetpointsList[ dofIndex ]->velocity = axisSetpointsList[ dofIndex ]->velocity;
    jointSetpointsList[ dofIndex ]->acceleration = axisSetpointsList[ dofIndex ]->acceleration;
    jointSetpointsList[ dofIndex ]->force = actuatorForceSetpoint;
  }
  
  fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, i=%.3f, d=%.3f, s=%.3f, vd=%.3f\n", axisSetpointsList[ 0 ]->position,
                                                                                          axisMeasuresList[ 0 ]->position,
                                                                                          axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                          axisMeasuresList[ 0 ]->inertia, axisMeasuresList[ 0 ]->damping, 
                                                                                          axisMeasuresList[ 0 ]->stiffness, axisSetpointsList[ 0 ]->velocity );
}
