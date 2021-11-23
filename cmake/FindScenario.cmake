find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import scenario; print(scenario.__path__[0])"
    OUTPUT_VARIABLE OUT_PATH
)
message(STATUS ${OUT_PATH})

find_library (
	ScenarioInstalled
	NAMES "Scenario"
	HINTS ${OUT_PATH}
	DOC "Scenario CPP Core library"
)

find_package(Scenario REQUIRED core HINTS OUT_PATH)
