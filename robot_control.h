///////////////////////////////////////////////////////////////////////////////////////
//                                                                                   //
//  Copyright (c) 2016-2018 Leonardo Consoni <consoni_2519@hotmail.com>              //
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
enum RobotState 
{ 
  ROBOT_PASSIVE,            ///< State for fully compliant robot control/behaviour
  ROBOT_OFFSET,             ///< State for definition of reference (zero) for controller measurements 
  ROBOT_CALIBRATION,        ///< State for definition of limits (min-max) for controller measurements 
  ROBOT_PREPROCESSING,      ///< State for custom automatic preprocessing of controller parameters 
  ROBOT_OPERATION,          ///< State for normal controller operation 
  ROBOT_STATES_NUMBER       ///< Total number of control states 
};

/// Control used variables list indexes enumeration
typedef struct RobotVariables
{
  double position, velocity, force, acceleration, inertia, stiffness, damping;
}
RobotVariables;

/// Robot control interface declaration macro, using [Plug-in Loader](https://github.com/LabDin/Plugin-Loader) convention
#define ROBOT_CONTROL_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( bool, Interface, InitController, const char* ) \
        INIT_FUNCTION( void, Interface, EndController, void ) \
        INIT_FUNCTION( size_t, Interface, GetJointsNumber, void ) \
        INIT_FUNCTION( const char**, Interface, GetJointNamesList, void ) \
        INIT_FUNCTION( size_t, Interface, GetAxesNumber, void ) \
        INIT_FUNCTION( const char**, Interface, GetAxisNamesList, void ) \
        INIT_FUNCTION( void, Interface, SetControlState, enum RobotState ) \
        INIT_FUNCTION( void, Interface, RunControlStep, RobotVariables**, RobotVariables**, RobotVariables**, RobotVariables**, double )
        
#endif  // ROBOT_CONTROL_H
    

/// @class ROBOT_CONTROL_INTERFACE 
/// @brief Robot control methods to be implemented by plugins
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn bool InitController( const char* configurationString )                                                                                
/// @brief Calls plugin specific robot controller initialization  
/// @param configurationString string containing the robot/plugin specific configuration
/// @return true on successful initialization, false otherwise  
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn void EndController( void )
/// @brief Calls plugin specific robot controller data deallocation                              
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void RunControlStep( RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )                                                                        
/// @brief Calls plugin specific logic to process single control pass and joints/axes coordinate conversions
/// @param[in,out] jointMeasuresList list of per degree-of-freedom control variables representing current robot joints measures                                    
/// @param[in,out] axisMeasuresList list of per degree-of-freedom control variables representing current robot effector measures                                             
/// @param[in,out] jointSetpointsList list of per degree-of-freedom control variables representing robot joints desired states
/// @param[in,out] axisSetpointsList list of per degree-of-freedom control variables representing robot effector desired states
/// @param[in] timeDelta time (in seconds) since the last control pass was called
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void SetControlState( enum RobotState controlState )
/// @brief Pass control state to trigger possible plugin specific behaviour
/// @param[in] controlState member of state enumeration defined in control_definitions.h
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetJointsNumber( void )
/// @brief Get number of joint coordinates/degrees-of-freedom for given robot
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn char** GetJointNamesList( void )
/// @brief Get names of all joints for given robot
/// @return list of joint name strings
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetAxesNumber( void )
/// @brief Get number of axis coordinates/degrees-of-freedom for given robot
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn char** GetAxisNamesList( RobotController controller )
/// @brief Get names of all axes for given robot
/// @return list of effector axis name strings
///
/// @memberof ROBOT_CONTROL_INTERFACE
