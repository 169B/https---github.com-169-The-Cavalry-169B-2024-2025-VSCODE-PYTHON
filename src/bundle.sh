#!/bin/bash

# Display a large ASCII banner
cat << "EOF"
 ████   ████████   ████████  ███████████ 
░░███  ███░░░░███ ███░░░░███░░███░░░░░███
 ░███ ░███   ░░░ ░███   ░███ ░███    ░███
 ░███ ░█████████ ░░█████████ ░██████████ 
 ░███ ░███░░░░███ ░░░░░░░███ ░███░░░░░███
 ░███ ░███   ░███ ███   ░███ ░███    ░███
 █████░░████████ ░░████████  ███████████ 
░░░░░  ░░░░░░░░   ░░░░░░░░  ░░░░░░░░░░░  
                                               
                                     
   169B. 2024-2025-HighStakesVEXV5
EOF

# 2-second intentional delay


# Cool loading spinner animation while bundling
echo "Bundling all the code... Please wait."

# Layered spinner patterns
spinner_1='█▓▒░'
spinner_2='▒▓░█'
spinner_3='░█▓▒'
spinner_4='▓░█▒'

# Start the layered spinner with alternating patterns
for i in {1..5}; do
    # Print multiple layers of patterns
    printf "\r${spinner_1:i%${#spinner_1}:1} ${spinner_2:i%${#spinner_2}:1} ${spinner_3:i%${#spinner_3}:1} ${spinner_4:i%${#spinner_4}:1}"  
    sleep 0.2
done

# Combine all the files into the final main.py
cat src/MAIN_GB/main1.py src/AUTO/autoselect.py src/OTHER/picture.py src/DRIVER_FUNCTIONS/drive.py src/DRIVER_FUNCTIONS/LB.py src/INIT/init.py src/OTHER/testcode.py src/PID_FORWARD/PID.py src/PID_FORWARD/NEWPID.py src/PID_TURN/PIDTURN.py src/AUTO/autonomous.py > src/main.py

# Output message
echo "Bundling complete: main.py has been generated and it's ready to upload!"
