%module(package="scenario.bindings") monopod

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/monopod/Joint.h"
#include "scenario/monopod/Model.h"
#include "scenario/monopod/World.h"
#include "scenario/core/Joint.h"
#include "scenario/core/Link.h"
#include "scenario/core/Model.h"
#include "scenario/core/World.h"
#include <cstdint>
%}

// https://stackoverflow.com/questions/30407792/how-to-create-an-extension-to-already-wrapped-library-via-swig/30413268#30413268

%naturalvar;

// Keep templated functions above the %rename directive
// %inline %{
// namespace scenario::monopod::utils {
//     template <typename Base, typename Derived>
//     std::shared_ptr<Derived> ToMonopod(const std::shared_ptr<Base>& base)
//     {
//         return std::dynamic_pointer_cast<Derived>(base);
//     }
// }
// %}

// Helpers for downcasting to monopod classes
// %template(ToMonopodWorld) scenario::monopod::utils::ToMonopod<scenario::core::World, scenario::monopod::World>;
// %template(ToMonopodModel) scenario::monopod::utils::ToMonopod<scenario::core::Model, scenario::monopod::Model>;
// %template(ToMonopodJoint) scenario::monopod::utils::ToMonopod<scenario::core::Joint, scenario::monopod::Joint>;

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
%rename("") Pose;
%rename("") Link;
%rename("") Joint;
%rename("") Model;
%rename("") World;
%rename("") Limit;
%rename("") Contact;
%rename("") JointType;
%rename("") JointLimit;
%rename("") ContactPoint;
%rename("") JointControlMode;

// Other templates for ScenarI/O APIs
%shared_ptr(scenario::monopod::Joint)
%shared_ptr(scenario::monopod::Model)
%shared_ptr(scenario::monopod::World)

// Ignored methods
// %ignore "scenario/monopod/easylogging++.h";

// %ignore "scenario/monopod/Joint.h";
// %ignore "scenario/monopod/Model.h";

// ScenarI/O headers
//%include "scenario/monopod/Joint.h"
%include "scenario/monopod/Model.h"
%include "scenario/monopod/World.h"
