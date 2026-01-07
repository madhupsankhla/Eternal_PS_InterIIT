from z_module import init_z_axis, z_axis, cleanup_z_axis
import time

# Initialize once at start
init_z_axis()

try:
    # Move 50 cm UP
    z_axis(15, 1)
    time.sleep(2)
    
    # Move 30 cm DOWN
    z_axis(15, 0)
    time.sleep(2)
    
finally:
    cleanup_z_axis()
