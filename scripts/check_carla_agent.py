"""
Check whether CARLA BasicAgent is available for import.
Reports CARLA_ROOT, path existence, and import success.
Run: python scripts/check_carla_agent.py
"""
import os
import sys


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)

    print("CARLA BasicAgent availability check")
    print("=" * 50)

    carla_root = os.environ.get("CARLA_ROOT")
    if carla_root:
        print(f"CARLA_ROOT is set: {carla_root}")
        agent_path = os.path.join(carla_root, "PythonAPI", "carla", "agents", "navigation", "basic_agent.py")
        if os.path.isfile(agent_path):
            print(f"  agents/navigation/basic_agent.py exists: yes")
        else:
            print(f"  agents/navigation/basic_agent.py exists: no")
    else:
        print("CARLA_ROOT is not set")

    vendored = os.path.join(repo_root, "third_party", "carla_pythonapi")
    if os.path.isdir(vendored):
        vendored_agent = os.path.join(vendored, "agents", "navigation", "basic_agent.py")
        print(f"Vendored path exists: {vendored}")
        print(f"  agents/navigation/basic_agent.py exists: {os.path.isfile(vendored_agent)}")
    else:
        print(f"Vendored path exists: no ({vendored})")

    print()
    print("Import test:")

    # Try vendored first (same order as closed_loop_drive will use)
    if os.path.isdir(vendored) and vendored not in sys.path:
        sys.path.insert(0, vendored)
    try:
        from agents.navigation.basic_agent import BasicAgent
        print("  OK (BasicAgent imported)")
        # Report which path provided it
        mod = __import__("agents.navigation.basic_agent", fromlist=["BasicAgent"])
        print(f"  Loaded from: {os.path.dirname(mod.__file__)}")
        sys.exit(0)
    except ImportError as e:
        pass

    # Try CARLA_ROOT/PythonAPI/carla
    if carla_root:
        api_path = os.path.join(carla_root, "PythonAPI", "carla")
        if os.path.isdir(api_path) and api_path not in sys.path:
            sys.path.insert(0, api_path)
        try:
            from agents.navigation.basic_agent import BasicAgent
            print("  OK (BasicAgent imported via CARLA_ROOT)")
            mod = __import__("agents.navigation.basic_agent", fromlist=["BasicAgent"])
            print(f"  Loaded from: {os.path.dirname(mod.__file__)}")
            sys.exit(0)
        except ImportError:
            pass

    # Try parent of carla package
    try:
        import carla
        carla_dir = os.path.dirname(os.path.abspath(carla.__file__))
        parent = os.path.dirname(carla_dir)
        if parent not in sys.path:
            sys.path.insert(0, parent)
        from agents.navigation.basic_agent import BasicAgent
        print("  OK (BasicAgent imported via carla package parent)")
        mod = __import__("agents.navigation.basic_agent", fromlist=["BasicAgent"])
        print(f"  Loaded from: {os.path.dirname(mod.__file__)}")
        sys.exit(0)
    except ImportError:
        pass

    print("  FAILED (BasicAgent could not be imported)")
    print()
    print("To enable route-following in closed_loop_drive.py, either:")
    print("  1. Set CARLA_ROOT to your CARLA install (e.g. /opt/carla) if it contains PythonAPI/carla/agents, or")
    print("  2. Use the vendored agents: ensure third_party/carla_pythonapi is populated from CARLA 0.9.16.")
    sys.exit(1)


if __name__ == "__main__":
    main()
