import re

# File paths
robot_states_path = "src/main/java/frc/robot/AbstractClasses/RobotStates.java"
ninjas_subsystem_path = "src/main/java/frc/robot/AbstractClasses/NinjasSubsystem.java"

# Read the current RobotStates enum values
with open(robot_states_path, 'r') as file:
    robot_states_content = file.read()

# Extract the enum values
enum_values = re.findall(r'\b(\w+),\s*(?=\n|\})', robot_states_content)

# Read the NinjasSubsystem content
with open(ninjas_subsystem_path, 'r') as file:
    ninjas_subsystem_content = file.read()

# Existing methods in NinjasSubsystem
existing_methods = re.findall(r'public abstract void (\w+)\(\);', ninjas_subsystem_content)

# Identify new states that need methods
new_methods = [state.lower() for state in enum_values if state.lower() not in existing_methods]

# Add new methods to NinjasSubsystem if any
if new_methods:
    methods_to_add = "\n".join([f"    public abstract void {method}();" for method in new_methods])
    methods_section = f"{methods_to_add}\n\n    @Override\n    public void periodic() {{"

    # Add new methods in the abstract class
    ninjas_subsystem_content = re.sub(r'(@Override\n    public void periodic\(\) {)', f'{methods_to_add}\n\n    \\1', ninjas_subsystem_content)

    # Add new cases in the switch statement
    for method in new_methods:
        case_to_add = f"            case {method.upper()}:\n              {method}();\n              break;"
        ninjas_subsystem_content = re.sub(r'(default:\n              idle\(\);\n              break;)', f'{case_to_add}\n          \\1', ninjas_subsystem_content)

    # Write the updated content back to NinjasSubsystem.java
    with open(ninjas_subsystem_path, 'w') as file:
        file.write(ninjas_subsystem_content)
