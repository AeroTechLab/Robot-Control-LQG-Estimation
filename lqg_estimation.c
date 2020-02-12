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
#define IMPEDANCE_SAMPLES_NUMBER_MAX 1000

enum ControlState controlState = CONTROL_PASSIVE;

const char* DOF_NAMES[ DOFS_NUMBER_MAX ] = { "angle1", "angle2", "angle3", "angle4", "angle5", "angle6" };

double samplingTime = 0.0;

double costRatio = 1.0;
double forceProportionalGain = 0.0, forceIntegralGain = 0.0, forceDifferentialGain = 0.0;
double interactionForceSetpoint = 0.0;

double inputsList[ 1 ] = { 0 };
double measuresList[ 1 ] = { 0 };
double statesList[ 3 ] = { 0 };
double loadImpedance[ 3 ] = { 0 };
double feedbacksList[ 1 ] = { 0 };

typedef struct DoFData
{
  LinearSystem linearSystem;
  KFilter observer;
  ILQRegulator regulator;

  double loadInertiaMin, loadDampingMin;
  double actuatorInertiaMin, actuatorDampingMin;
  size_t impedanceSamplesCount;
  
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
  costRatio = fmax( strtod( strtok( NULL, " " ), NULL ), 0.0 );
  forceProportionalGain = fmax( strtod( strtok( NULL, " " ), NULL ), 0.0 );
  forceIntegralGain = fmax( strtod( strtok( NULL, " " ), NULL ), 0.0 );
  forceDifferentialGain = fmin( strtod( strtok( NULL, " " ), NULL ), 0.0 );  

  for( size_t dofIndex = 0; dofIndex < dofsNumber; dofIndex++ )
  {
    DoFData* dof = &(dofsList[ dofIndex ]);
    
    dof->linearSystem = SystemLinearizer_CreateSystem( 3, 1, LINEARIZATION_MAX_SAMPLES );
    dof->observer = Kalman_CreateFilter( 3, 1, 1 );
    dof->regulator = ILQR_Create( 3, 1, costRatio );
    
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
      dof->actuatorDampingMin = 0.0;
      dof->actuatorInertiaMin = 0.1;
      dof->loadDampingMin = 0.0;
      dof->loadInertiaMin = 0.1;
    }
    
    dof->lastForceError = 0.0;
    dof->impedanceSamplesCount = 0;
    
    Kalman_Reset( dof->observer );
  }
  
  samplingTime = 0.0;
  
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
    double velocitySetpoint = 0.0;
    
    if( controlState != CONTROL_OFFSET )
    {
      // e = x - x^d = x_h - x^d = x_r = x^d
      measuresList[ 0 ] = axisMeasuresList[ dofIndex ]->position - axisSetpointsList[ dofIndex ]->position;
      feedbacksList[ 0 ] = 0.0;
      
      axisMeasuresList[ dofIndex ]->damping = fmax( axisMeasuresList[ dofIndex ]->damping, dof->actuatorDampingMin );
      axisMeasuresList[ dofIndex ]->inertia = fmax( axisMeasuresList[ dofIndex ]->inertia, dof->actuatorInertiaMin );
      
      // f_int = -K_h * (-e) + D_h * dot(x) + M_h * ddot(x)
      statesList[ 0 ] = measuresList[ 0 ];
      statesList[ 1 ] = axisMeasuresList[ dofIndex ]->velocity;
      statesList[ 2 ] = axisMeasuresList[ dofIndex ]->acceleration;
      inputsList[ 0 ] = jointSetpointsList[ dofIndex ]->force;//axisMeasuresList[ dofIndex ]->force;
      if( SystemLinearizer_AddSample( dof->linearSystem, statesList, inputsList ) >= LINEARIZATION_MAX_SAMPLES )
        SystemLinearizer_Identify( dof->linearSystem, loadImpedance );
      
      loadImpedance[ 0 ] = fmax( fmin( loadImpedance[ 0 ], 100.0 ), 0.0 );
      loadImpedance[ 1 ] = fmax( fmin( loadImpedance[ 1 ], 100.0 ), 0.0 );
      loadImpedance[ 2 ] = fmax( fmin( loadImpedance[ 2 ], 10.0 ), 0.1 );      

      if( controlState == CONTROL_CALIBRATION ) 
      {
        if( dof->impedanceSamplesCount++ < IMPEDANCE_SAMPLES_NUMBER_MAX )
        {
          feedbacksList[ 0 ] = 10.0 * sin( samplingTime );
       
          dof->actuatorDampingMin += axisMeasuresList[ dofIndex ]->damping / IMPEDANCE_SAMPLES_NUMBER_MAX;
          dof->actuatorInertiaMin += axisMeasuresList[ dofIndex ]->inertia / IMPEDANCE_SAMPLES_NUMBER_MAX;
        
          dof->loadDampingMin += loadImpedance[ 1 ] / IMPEDANCE_SAMPLES_NUMBER_MAX;
          dof->loadInertiaMin += loadImpedance[ 2 ] / IMPEDANCE_SAMPLES_NUMBER_MAX;
        }
        
        loadImpedance[ 1 ] = dof->loadDampingMin;
        loadImpedance[ 2 ] = dof->loadInertiaMin;
      }
      else
      {
        loadImpedance[ 1 ] = fmax( loadImpedance[ 1 ], dof->loadDampingMin );
        loadImpedance[ 2 ] = fmax( loadImpedance[ 2 ], dof->loadInertiaMin );
        // ddot(x) = u * 1/M_h - D_h/M_h * dot(x)
        Kalman_SetTransitionFactor( dof->observer, 2, 1, -loadImpedance[ 1 ] / loadImpedance[ 2 ] );
        ILQR_SetTransitionFactor( dof->regulator, 2, 1, -loadImpedance[ 1 ] / loadImpedance[ 2 ] );
        Kalman_SetInputFactor( dof->observer, 2, 0, 1.0 / loadImpedance[ 2 ] );
        ILQR_SetInputFactor( dof->regulator, 2, 0, 1.0 / loadImpedance[ 2 ] );
        // u = -K_h * e + f_int
        inputsList[ 0 ] = -loadImpedance[ 0 ] * measuresList[ 0 ] + axisMeasuresList[ 0 ]->force;
        // z = Az + Bu + K( e - C( Az + Bu ) )
        Kalman_Predict( dof->observer, inputsList, statesList );
        Kalman_Update( dof->observer, measuresList, statesList );
        // f_lqg = -Gz
        if( controlState == CONTROL_OPERATION ) ILQR_CalculateFeedback( dof->regulator, statesList, feedbacksList );
      }
        
      // f_int^d = f_lqg + f_fb
      interactionForceSetpoint = -feedbacksList[ 0 ] + axisSetpointsList[ 0 ]->force;
      // f_r^d = M_r * ddot(x) + D_r * dot(x) - (-f_int^d)
      actuatorForceSetpoint = axisMeasuresList[ dofIndex ]->inertia * axisMeasuresList[ dofIndex ]->acceleration
                              + axisMeasuresList[ dofIndex ]->damping * axisMeasuresList[ dofIndex ]->velocity
                              + interactionForceSetpoint;
      // Force-velocity PI control (SEA)
      double forceError = interactionForceSetpoint - axisMeasuresList[ dofIndex ]->force;
      //dof->velocitySetpoint += forceProportionalGain * ( forceError - dof->lastForceError ) + forceIntegralGain * deltaTime * forceError;
      dof->velocitySetpoint = forceProportionalGain * forceError 
                              + forceDifferentialGain * ( forceError - dof->lastForceError ) / deltaTime
                              + forceIntegralGain * ( forceError + dof->lastForceError ) * deltaTime; 
      dof->lastForceError = forceError;

      axisMeasuresList[ dofIndex ]->stiffness = loadImpedance[ 0 ];
      axisMeasuresList[ dofIndex ]->damping = loadImpedance[ 1 ];
      axisMeasuresList[ dofIndex ]->inertia = loadImpedance[ 2 ];
    }
    
    axisSetpointsList[ dofIndex ]->velocity = dof->velocitySetpoint;
    
    jointSetpointsList[ dofIndex ]->position = axisSetpointsList[ dofIndex ]->position;
    jointSetpointsList[ dofIndex ]->velocity = axisSetpointsList[ dofIndex ]->velocity;
    jointSetpointsList[ dofIndex ]->acceleration = axisSetpointsList[ dofIndex ]->acceleration;
    jointSetpointsList[ dofIndex ]->force = interactionForceSetpoint;
  }
  
  fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, i=%.3f, d=%.3f, s=%.3f, vd=%.3f\n", axisSetpointsList[ 0 ]->position, axisMeasuresList[ 0 ]->position,
                                                                                          axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                          axisMeasuresList[ 0 ]->inertia, axisMeasuresList[ 0 ]->damping, 
                                                                                          axisMeasuresList[ 0 ]->stiffness, axisSetpointsList[ 0 ]->velocity );
}
