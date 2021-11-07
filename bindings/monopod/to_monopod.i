%pythonbegin %{
from typing import Union
%}

%extend scenario::core::World {
  %pythoncode %{
    def to_monopod(self) -> Union["scenario.bindings.monopod.World", "scenario.bindings.core.World"]:
        import scenario.bindings.monopod
        return scenario.bindings.monopod.ToMonopodWorld(self)
  %}
}

%extend scenario::core::Model {
  %pythoncode %{
    def to_monopod(self) -> Union["scenario.bindings.monopod.Model", "scenario.bindings.core.Model"]:
        import scenario.bindings.monopod
        return scenario.bindings.monopod.ToMonopodModel(self)
  %}
}

%extend scenario::core::Joint {
  %pythoncode %{
    def to_monopod(self) -> Union["scenario.bindings.monopod.Joint", "scenario.bindings.core.Joint"]:
        import scenario.bindings.monopod
        return scenario.bindings.monopod.ToMonopodJoint(self)
  %}
}
