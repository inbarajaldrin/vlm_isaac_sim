"""
Template Extension for Isaac Sim

This file contains the main extension class that will be loaded by Isaac Sim.
"""

import omni.ext
import omni.ui as ui
import carb
import omni.usd
from typing import Dict, Any, List, Optional, Union


# Extension Methods required by Omniverse Kit
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtensionTemplate(omni.ext.IExt):
    def __init__(self) -> None:
        """Initialize the extension."""
        super().__init__()
        self.ext_id = None
        self._usd_context = None
        self._window = None
        self._settings = carb.settings.get_settings()
        self._button_click_count = 0
        

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        print("trigger on_startup for: ", ext_id)
        print("settings: ", self._settings.get("/exts/omni.kit.pipapi"))
        
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()
        
        # Create UI window with button
        self._create_ui()
        
        # Initialize your extension here
        self._initialize_extension()
    
    def on_shutdown(self):
        print("trigger on_shutdown for: ", self.ext_id)
        
        # Clean up resources
        if self._window:
            self._window = None
            
        # Clean up your extension resources here
        self._cleanup_extension()
    
    def _initialize_extension(self):
        """
        Initialize your extension logic here.
        This is where you would set up your extension's functionality.
        """
        print("Initializing extension...")
        
        # Example: Get the current stage
        stage = self._usd_context.get_stage()
        if stage:
            print(f"Current stage: {stage.GetRootLayer().realPath}")
        else:
            print("No active stage")
            
        # Add your initialization logic here
        
    def _cleanup_extension(self):
        """
        Clean up your extension resources here.
        """
        print("Cleaning up extension...")
        
        # Add your cleanup logic here
    
    def _create_ui(self):
        """Create the extension UI window with button."""
        try:
            # Create a window for the extension
            self._window = ui.Window("My Extension Template", width=300, height=200)
            
            with self._window.frame:
                with ui.VStack():
                    # Title
                    ui.Label("My Extension Template", style={"fontSize": 18, "color": ui.color.white})
                    ui.Separator()
                    
                    # Status label
                    self._status_label = ui.Label("Extension loaded successfully!", style={"color": ui.color.green})
                    
                    # Button
                    ui.Button("Click Me!", clicked_fn=self._on_button_clicked, style={"margin": 10})
                    
                    # Click counter
                    self._counter_label = ui.Label(f"Button clicked: {self._button_click_count} times")
                    
                    ui.Spacer()
                    
                    # Info section
                    with ui.CollapsableFrame("Extension Info"):
                        with ui.VStack():
                            ui.Label(f"Extension ID: {self.ext_id or 'Not loaded'}")
                            ui.Label(f"USD Context: {'Available' if self._usd_context else 'Not available'}")
                            
        except Exception as e:
            carb.log_error(f"Failed to create UI: {e}")
    
    def _on_button_clicked(self):
        """Handle button click events."""
        self._button_click_count += 1
        print(f"Button clicked! Count: {self._button_click_count}")
        
        # Update the counter label
        if hasattr(self, '_counter_label'):
            self._counter_label.text = f"Button clicked: {self._button_click_count} times"
        
        # Update status
        if hasattr(self, '_status_label'):
            self._status_label.text = f"Button clicked {self._button_click_count} times!"
            self._status_label.style = {"color": ui.color.blue}
    
    # Add your custom methods here
    def my_custom_method(self, param1: str, param2: int = 0) -> Dict[str, Any]:
        """
        Example custom method.
        
        Args:
            param1: First parameter
            param2: Second parameter with default value
            
        Returns:
            Dictionary with result
        """
        try:
            # Your custom logic here
            result = {
                "status": "success",
                "message": f"Custom method called with {param1} and {param2}",
                "param1": param1,
                "param2": param2
            }
            
            return result
            
        except Exception as e:
            return {
                "status": "error",
                "message": str(e)
            }

