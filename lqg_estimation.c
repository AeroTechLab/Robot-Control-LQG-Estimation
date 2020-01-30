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
double loadImpedancesList[ 3 ] = { 0 };
double feedbacksList[ 1 ] = { 0 };

typedef struct DoFData
{
  LinearSystem linearSystem;
  KFilter observer;
  ILQRegulator regulator;

  double loadImpedancesMinList[ 3 ];
  double actuatorImpedancesMinList[ 3 ];
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
      dof->actuatorImpedancesMinList[ 0 ] = 0.0;
      dof->actuatorImpedancesMinList[ 1 ] = 0.0;
      dof->actuatorImpedancesMinList[ 2 ] = 0.1;
      dof->loadImpedancesMinList[ 0 ] = 0.0;
      dof->loadImpedancesMinList[ 1 ] = 0.0;
      dof->loadImpedancesMinList[ 2 ] = 0.1;
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
    
    if( controlState != CONTROL_OFFSET )
    {
      if( controlState == CONTROL_CALIBRATION ) axisSetpointsList[ 0 ]->position = sin( 2 * M_PI * samplingTime / 4 ) / 2;
      // e = x - x^d = x_h - x^d = x_r = x^d
      measuresList[ 0 ] = axisMeasuresList[ dofIndex ]->position - axisSetpointsList[ dofIndex ]->position;
      feedbacksList[ 0 ] = ( controlState != CONTROL_PASSIVE ) ? positionProportionalGain * measuresList[ 0 ] : 0.0;
      
      axisMeasuresList[ dofIndex ]->stiffness = fmax( axisMeasuresList[ dofIndex ]->stiffness, dof->actuatorImpedancesMinList[ 0 ] );
      axisMeasuresList[ dofIndex ]->damping = fmax( axisMeasuresList[ dofIndex ]->damping, dof->actuatorImpedancesMinList[ 1 ] );
      axisMeasuresList[ dofIndex ]->inertia = fmax( axisMeasuresList[ dofIndex ]->inertia, dof->actuatorImpedancesMinList[ 2 ] );
      
      if( identificationEnabled )
      {
        // -f_int = K_h * ( x^d - x ) + D_h * dot(x) + M_h * ddot(x)
        statesList[ 0 ] = -measuresList[ 0 ];
        statesList[ 1 ] = axisMeasuresList[ dofIndex ]->velocity;
        statesList[ 2 ] = axisMeasuresList[ dofIndex ]->acceleration;
        inputsList[ 0 ] = -axisMeasuresList[ 0 ]->force;
        if( SystemLinearizer_AddSample(  dof->linearSystem, inputsList, statesList ) >= LINEARIZATION_MAX_SAMPLES )
          SystemLinearizer_Identify( dof->linearSystem, loadImpedancesList );
        
        loadImpedancesList[ 0 ] = fmax( 0.0, loadImpedancesList[ 0 ] );
        loadImpedancesList[ 1 ] = fmax( 0.0, loadImpedancesList[ 1 ] );
        loadImpedancesList[ 2 ] = fmax( 0.1, loadImpedancesList[ 2 ] );
        
        if( controlState == CONTROL_CALIBRATION ) 
        {       
          dof->actuatorImpedancesMinList[ 0 ] = ( axisMeasuresList[ dofIndex ]->stiffness + dof->actuatorImpedancesMinList[ 0 ] ) / 2;
          dof->actuatorImpedancesMinList[ 1 ] = ( axisMeasuresList[ dofIndex ]->damping + dof->actuatorImpedancesMinList[ 1 ] ) / 2;
          dof->actuatorImpedancesMinList[ 2 ] = ( axisMeasuresList[ dofIndex ]->inertia + dof->actuatorImpedancesMinList[ 2 ] ) / 2;
          
          dof->loadImpedancesMinList[ 0 ] = ( loadImpedancesList[ 0 ] + dof->loadImpedancesMinList[ 0 ] ) / 2;
          dof->loadImpedancesMinList[ 1 ] = ( loadImpedancesList[ 1 ] + dof->loadImpedancesMinList[ 1 ] ) / 2;
          dof->loadImpedancesMinList[ 2 ] = ( loadImpedancesList[ 2 ] + dof->loadImpedancesMinList[ 2 ] ) / 2;
        }
        else if( controlState == CONTROL_OPERATION )
        {
          loadImpedancesList[ 0 ] = fmax( loadImpedancesList[ 0 ], dof->loadImpedancesMinList[ 0 ] );
          loadImpedancesList[ 1 ] = fmax( loadImpedancesList[ 1 ], dof->loadImpedancesMinList[ 1 ] );
          loadImpedancesList[ 2 ] = fmax( loadImpedancesList[ 2 ], dof->loadImpedancesMinList[ 2 ] );
          // ddot(x) = u * 1/M_h - D_h/M_h * dot(x)
          Kalman_SetTransitionFactor( dof->observer, 2, 1, -loadImpedancesList[ 1 ] / loadImpedancesList[ 2 ] );
          ILQR_SetTransitionFactor( dof->regulator, 2, 1, -loadImpedancesList[ 1 ] / loadImpedancesList[ 2 ] );
          Kalman_SetInputFactor( dof->observer, 2, 0, 1.0 / loadImpedancesList[ 2 ] );
          ILQR_SetInputFactor( dof->regulator, 2, 0, 1.0 / loadImpedancesList[ 2 ] );
          // u = -K_h * ( x - x^d ) + (-f_int)
          inputsList[ 0 ] = -loadImpedancesList[ 0 ] * measuresList[ 0 ] - axisMeasuresList[ 0 ]->force;
          // z = Az + Bu + K( x - x^d - C( Az + Bu ) )
          Kalman_Predict( dof->observer, inputsList, statesList );
          Kalman_Update( dof->observer, measuresList, statesList );
          // f_lqg = -Gz
          ILQR_CalculateFeedback( dof->regulator, statesList, feedbacksList );
        }
      }
      // (-f_int^d) = f_lqg + f_fb
      interactionForceSetpoint = -feedbacksList[ 0 ] + axisSetpointsList[ 0 ]->force;
      // f_r^d = M_r * ddot(x) + D_r * dot(x) - (-f_int^d)
      actuatorForceSetpoint = axisMeasuresList[ dofIndex ]->inertia * axisMeasuresList[ dofIndex ]->acceleration
                              + axisMeasuresList[ dofIndex ]->damping * axisMeasuresList[ dofIndex ]->velocity
                              + interactionForceSetpoint;
      
      axisMeasuresList[ dofIndex ]->stiffness = loadImpedancesList[ 0 ];
      axisMeasuresList[ dofIndex ]->damping = loadImpedancesList[ 1 ];
      axisMeasuresList[ dofIndex ]->inertia = loadImpedancesList[ 2 ];
    }
    
    jointSetpointsList[ dofIndex ]->position = axisSetpointsList[ dofIndex ]->position;
    jointSetpointsList[ dofIndex ]->velocity = axisSetpointsList[ dofIndex ]->velocity;
    jointSetpointsList[ dofIndex ]->acceleration = axisSetpointsList[ dofIndex ]->acceleration;
    jointSetpointsList[ dofIndex ]->force = actuatorForceSetpoint;
  }
  
  fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, i=%.3f, d=%.3f, s=%.3f, vd=%.3f\n", axisSetpointsList[ 0 ]->position, axisMeasuresList[ 0 ]->position,
                                                                                          axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                          axisMeasuresList[ 0 ]->inertia, axisMeasuresList[ 0 ]->damping, 
                                                                                          axisMeasuresList[ 0 ]->stiffness, axisSetpointsList[ 0 ]->velocity );
}
