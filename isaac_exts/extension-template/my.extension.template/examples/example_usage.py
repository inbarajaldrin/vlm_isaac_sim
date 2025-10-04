"""
Example usage of the MyExtensionTemplate.

This script demonstrates how to use the extension programmatically.
The extension now includes a UI with buttons for interactive control.
"""

import omni.ext
import omni.usd
import omni.kit.commands
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path


def example_usage():
    """
    Example of how to use the template extension.
    The extension now includes a UI window with buttons.
    """
    print("Example usage of MyExtensionTemplate")
    print("The extension includes a UI window with the following features:")
    print("- Main 'Click Me!' button with click counter")
    print("- Status updates and extension info")
    print("- Simple and clean interface")
    
    # Get the current USD context
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    if stage:
        print(f"Current stage: {stage.GetRootLayer().realPath}")
        
        # Example: List all prims in the scene
        prims = list(stage.Traverse())
        print(f"Found {len(prims)} prims in the scene")
        
        for prim in prims[:5]:  # Show first 5 prims
            print(f"  - {prim.GetPath()}: {prim.GetTypeName()}")
    else:
        print("No active stage")
    
    # Example: Create objects programmatically (same as the UI buttons)
    try:
        print("Creating objects programmatically...")
        omni.kit.commands.execute("CreatePrim", prim_type="Sphere")
        print("Created a sphere")
        
        omni.kit.commands.execute("CreatePrim", prim_type="Cube")
        print("Created a cube")
        
    except Exception as e:
        print(f"Failed to create objects: {e}")


def demonstrate_ui_features():
    """
    Demonstrate the UI features available in the extension.
    """
    print("\n=== Extension UI Features ===")
    print("When the extension is loaded, you'll see a window with:")
    print("1. Title: 'My Extension Template'")
    print("2. Status label showing current state")
    print("3. 'Click Me!' button - increments counter on each click")
    print("4. Click counter display")
    print("5. Extension Info (collapsible):")
    print("   - Extension ID")
    print("   - USD Context status")
    print("\nThe button provides visual feedback through status updates!")


if __name__ == "__main__":
    example_usage()
    demonstrate_ui_features()

