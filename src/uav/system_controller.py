import asyncio
from pathlib import Path
import os
import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import load_state, update_state, STATE_FILE 

#THis file is when we wanted to reboot or shutdown wirelessly and via commands, we might be removing this command system so 
# this file might be outdated, however it is pretty useful so id keep it around for now

STATE_POLL_HZ = 2.0 #How often to check state file

async def main():

    state_period_s = 1.0 / STATE_POLL_HZ

    print(f"State file: {STATE_FILE}")
    print()
    print("System Controller ready... (shutdown or reboot)")
    print()


    while True:
        state = load_state()
        action = state.get("pending_action")

        if action == "reboot":
            update_state("pending_action", None)
            update_state("last_command", "reboot")
            print("Rebooting...")
            await asyncio.sleep(0.5)
            os.system("sudo reboot")
            
        elif action == "shutdown":
            update_state("pending_action", None)
            update_state("last_command", "shutdown")
            print("Shutting Down...")
            await asyncio.sleep(0.5)
            os.system("sudo shutdown now")

        await asyncio.sleep(state_period_s)

if __name__ == "__main__":
    asyncio.run(main())