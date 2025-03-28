import traci
import sys
import os

# Configure path to SUMO if needed
# if 'SUMO_HOME' in os.environ:
#     tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
#     sys.path.append(tools)
# else:
#     sys.exit("Please declare environment variable 'SUMO_HOME'")

# Start SUMO with the configuration file
sumo_cmd = ["sumo", "-c", "1k.sumocfg", "--step-length", "0.1"]
traci.start(sumo_cmd)

# FollowerStopper controller parameters
U = 40  # Desired velocity (m/s) - optimal value from Experiment A
dx_0_1 = 4.5  # First region boundary intercept (m)
dx_0_2 = 5.25  # Second region boundary intercept (m)
dx_0_3 = 6.0  # Third region boundary intercept (m)
d_1 = 1.5  # First region deceleration rate (m/s²)
d_2 = 1.0  # Second region deceleration rate (m/s²)
d_3 = 0.5  # Third region deceleration rate (m/s²)

# Autonomous vehicle ID
av_id = "av_0"  # Match this to the ID in the routes file

# Control loop
step = 0
max_steps = 6000  # 600 seconds at 0.1s step length
print("Starting FollowerStopper controller simulation...")

while step < max_steps:
    traci.simulationStep()
    
    # Check if the autonomous vehicle exists in the simulation
    try:
        if av_id not in traci.vehicle.getIDList():
            print(f"Vehicle {av_id} not found in simulation")
            step += 1
            continue
        
        # Get leader info
        leader_info = traci.vehicle.getLeader(av_id)
        if leader_info is None:
            # No leader detected, maintain desired speed
            traci.vehicle.setSpeed(av_id, U)
            step += 1
            continue
        
        leader_id, gap = leader_info
        
        # Get speeds
        v_av = traci.vehicle.getSpeed(av_id)
        v_lead = traci.vehicle.getSpeed(leader_id)
        delta_v = v_lead - v_av  # Velocity difference (negative when AV is faster)
        
        # Compute delta_v_minus (negative arm of velocity difference)
        delta_v_minus = min(delta_v, 0)
        
        # Calculate region boundaries using the parabolas defined in the paper
        dx_1 = dx_0_1 + 0.5 * delta_v_minus**2 / d_1
        dx_2 = dx_0_2 + 0.5 * delta_v_minus**2 / d_2
        dx_3 = dx_0_3 + 0.5 * delta_v_minus**2 / d_3
        
        # Calculate commanded velocity using FollowerStopper logic
        if gap <= dx_1: 
            # Stopping region
            v_cmd = 0
        elif gap <= dx_2:
            # First adaptation region
            v = min(max(v_lead, 0), U)
            v_cmd = v * (gap - dx_1) / (dx_2 - dx_1)
        elif gap <= dx_3:
            # Second adaptation region
            v = min(max(v_lead, 0), U)
            v_cmd = v + (U - v) * (gap - dx_2) / (dx_3 - dx_2)
        else:
            # Safe region
            v_cmd = U
        
        # Apply the commanded velocity to the autonomous vehicle
        traci.vehicle.setSpeed(av_id, v_cmd)
        
        # Print status every 100 steps
        if step % 100 == 0:
            print(f"Step {step}: gap = {gap:.2f}m, v_av = {v_av:.2f}m/s, v_lead = {v_lead:.2f}m/s, v_cmd = {v_cmd:.2f}m/s")
    
    except traci.exceptions.TraCIException as e:
        print(f"TraCI exception: {e}")
    
    step += 1

# Close TraCI connection
traci.close()
print("Simulation completed.")