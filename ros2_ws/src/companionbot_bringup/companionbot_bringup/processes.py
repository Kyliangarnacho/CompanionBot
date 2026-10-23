"""One bringup policy: a required process exit ends its launch, preserving failure."""
from launch.actions import EmitEvent
from launch.events import Shutdown


def exit_with_runtime(event, context):
    if context.is_shutdown:
        return []
    if event.returncode:
        raise RuntimeError(f"Required runtime exited with code {event.returncode}")
    return [EmitEvent(event=Shutdown(reason="required runtime ended"))]
