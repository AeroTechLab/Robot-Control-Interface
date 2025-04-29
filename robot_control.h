///////////////////////////////////////////////////////////////////////////////////////
//                                                                                   //
//  Copyright (c) 2016-2020 Leonardo Consoni <leonardojc@protonmail.com>             //
//                                                                                   //
//  This file is part of Robot Control Interface.                                    //
//                                                                                   //
//  Robot Control Interface is free software: you can redistribute it and/or modify  //
//  it under the terms of the GNU Lesser General Public License as published         //
//  by the Free Software Foundation, either version 3 of the License, or             //
//  (at your option) any later version.                                              //
//                                                                                   //
//  Robot Control Interface is distributed in the hope that it will be useful,       //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                   //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                     //
//  GNU Lesser General Public License for more details.                              //
//                                                                                   //
//  You should have received a copy of the GNU Lesser General Public License         //
//  along with Robot Control Interface. If not, see <http://www.gnu.org/licenses/>.  //
//                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////


/// @file robot_control.h
/// @brief Generic robot control functions
///
/// Common robot control interface to be implemented by device specific plugins
/// Considers 2 different coordinate/degree-of-freedom sets: joints and axes (for a detailed explanation, see [The Joint/Axis Rationale]())

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159      ///< Defines mathematical Pi value if standard math.h one is not available
#endif

#include "plugin_loader/loader_macros.h"

/// Defined possible control states enumeration. Passed to generic or plugin specific robot control implementations
enum ControlState 
{ 
  CONTROL_PASSIVE,            ///< State for fully compliant robot control/behaviour
  CONTROL_OFFSET,             ///< State for definition of reference (zero) for controller measurements 
  CONTROL_CALIBRATION,        ///< State for definition of limits (min-max) for controller measurements 
  CONTROL_PREPROCESSING,      ///< State for custom automatic preprocessing of controller parameters 
  CONTROL_OPERATION,          ///< State for normal controller operation 
  CONTROL_STATES_NUMBER       ///< Total number of control states 
};

/// Control used variables list indexes enumeration
typedef struct DoFVariables
{
  double position, velocity, force, acceleration, inertia, stiffness, damping;
}
DoFVariables;

/// Robot control interface declaration macro, using [Plug-in Loader](https://github.com/AeroTechLab/Plugin-Loader) convention
#define ROBOT_CONTROL_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( bool, Interface, InitController, const char* ) \
        INIT_FUNCTION( void, Interface, EndController, void ) \
        INIT_FUNCTION( size_t, Interface, GetJointsNumber, void ) \
        INIT_FUNCTION( const char**, Interface, GetJointNamesList, void ) \
        INIT_FUNCTION( size_t, Interface, GetAxesNumber, void ) \
        INIT_FUNCTION( const char**, Interface, GetAxisNamesList, void ) \
        INIT_FUNCTION( void, Interface, SetControlState, enum ControlState ) \
        INIT_FUNCTION( void, Interface, RunControlStep, DoFVariables**, DoFVariables**, DoFVariables**, DoFVariables**, double ) \
        INIT_FUNCTION( size_t, Interface, GetExtraInputsNumber, void ) \
        INIT_FUNCTION( void, Interface, SetExtraInputsList, double* ) \
        INIT_FUNCTION( size_t, Interface, GetExtraOutputsNumber, void ) \
        INIT_FUNCTION( void, Interface, GetExtraOutputsList, double* )
        
#endif  // ROBOT_CONTROL_H
    

/// @class ROBOT_CONTROL_INTERFACE 
/// @brief Robot control methods to be implemented by plugins
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn bool InitController( const char* configurationString )                                                                                
/// @brief Calls plugin specific robot controller initialization  
/// @param[in] configurationString string containing the robot/plugin specific configuration       
/// @return true on successful initialization, false otherwise  
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn void EndController( void )
/// @brief Calls plugin specific robot controller data deallocation                              
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )                                                                        
/// @brief Calls plugin specific logic to process single control pass and joints/axes coordinate conversions
/// @param[in,out] jointMeasuresList list of per degree-of-freedom control variables representing current robot joints measures                                    
/// @param[in,out] axisMeasuresList list of per degree-of-freedom control variables representing current robot effector measures                                             
/// @param[in,out] jointSetpointsList list of per degree-of-freedom control variables representing robot joints desired states
/// @param[in,out] axisSetpointsList list of per degree-of-freedom control variables representing robot effector desired states
/// @param[in] timeDelta time (in seconds) since the last control pass was called
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void SetControlState( enum ControlState controlState )
/// @brief Pass control state to trigger possible plugin specific behaviour
/// @param[in] controlState member of state enumeration defined in control_definitions.h
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetJointsNumber( void )
/// @brief Get plugin specific number of joint coordinates/degrees-of-freedom
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn const char** GetJointNamesList( void )
/// @brief Get plugin specific names of all joints
/// @return list of joint name strings
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetAxesNumber( void )
/// @brief Get plugin specific number of axis coordinates/degrees-of-freedom
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn const char** GetAxisNamesList( void )
/// @brief Get plugin specific names of all axes
/// @return list of effector axis name strings
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetExtraInputsNumber( void )
/// @brief Get number of additional inputs needed for the robot control
/// @return number of additional inputs
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void SetExtraInputsList( double* inputsList )
/// @brief Set list of additional inputs for the next robot control step
/// @param[in] inputsList reference/pointer to list of addtional input values
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetExtraOutputsNumber( void )
/// @brief Get number of additional outputs provided by the robot control
/// @return number of additional outputs
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void GetExtraOutputsList( double* outputsList )
/// @brief Get list of additional outputs from the last robot control step
/// @param[in,out] outputsList reference/pointer to list of addtional output values
///
/// @memberof ROBOT_CONTROL_INTERFACE
