%module(package="scenario.bindings") monopod

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/monopod/Joint.h"
#include "scenario/monopod/Model.h"
#include "scenario/monopod/World.h"
#include <cstdint>
%}

// %naturalvar

// STL classes
%include <stdint.i>
%include <std_pair.i>
%include <std_array.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

// Import the module with core classes
// From http://www.swig.org/Doc4.0/Modules.html
%import "../core/core.i"

// NOTE: Keep all template instantiations above.
// Rename all methods to undercase with _ separators excluding the classes.
%rename("%(undercase)s") "";
%rename("") PID;
%rename("") Joint;
%rename("") Model;
%rename("") World;
%rename("") Limit;
%rename("") JointType;
%rename("") JointLimit;
%rename("") PhysicsEngine;
%rename("") JointControlMode;

// Other templates for ScenarI/O APIs
%shared_ptr(scenario::monopod::Joint)
%shared_ptr(scenario::monopod::Model)
%shared_ptr(scenario::monopod::World)

// Ignored methods
%ignore "scenario/monopod/easylogging++.h";


// ScenarI/O headers
%include "scenario/monopod/Joint.h"
%include "scenario/monopod/Model.h"
%include "scenario/monopod/World.h"
