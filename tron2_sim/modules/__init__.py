"""SimModule: composition protocol for variant-specific parts.

A module is an optional capability that brings its own I/O and control law
(e.g. the 2F gripper). Returning False from attach() means the model lacks that
capability (no grasper actuators, say); the core then drops the module and
everything else keeps working.

Threading: on_control / on_manual / on_publish / on_reset run on the physics
thread; on_render runs on the render thread and may only touch render_data and
viewer.user_scn (which the core clears each frame for modules to append overlay
geometry to). SDK push callbacks run concurrently with the physics thread, so
commands must always be applied as "build a new buffer, then swap the reference".
"""


class SimModule:
    name = "module"
    #: True -> in manual (paused) mode the core drives this module's actuators
    #: from the viewer sliders (ctrlrange swapped to joint limits, render_data.ctrl
    #: seeded). False -> the module idles in manual mode.
    slider_control = False

    def attach(self, core) -> bool:
        """Resolve indices/config and set up I/O. False = unavailable, core drops it."""
        return False

    def owned_actuators(self):
        """Actuator ids this module exclusively writes to data.ctrl (audited by the core)."""
        return []

    # ---- physics-thread hooks ----
    def on_control(self, core, dt):
        """Auto mode, before mj_step: write ctrl for owned_actuators."""

    def on_manual(self, core):
        """Manual mode, before mj_forward. Idles by default."""

    def on_publish(self, core):
        """State publishing stage (module-owned topics)."""

    def on_reset(self, core):
        """After the physics thread adopts a full reset; re-seed internal targets."""

    # ---- render-thread hook ----
    def on_render(self, core, render_data):
        """Before the viewer frame sync; may only touch render_data."""
