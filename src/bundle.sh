#!/bin/bash

# Display a large ASCII banner
cat << "EOF"
  ____    _____   ____   ____    ____    
 | __ )  | ____| |  _ \ |  _ \  |  _ \   
 |  _ \  |  _|   | |_) || |_) | | |_) |  
 | |_) | | |___  |  __/ |  __/  |  __/   
 |____/  |_____| |_|    |_|     |_|      
                                     
   169B
EOF

# 2-second intentional delay


# Cool loading spinner animation while bundling
echo "Bundling all the awesome code... Please wait."

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
cat src/MAIN_GB/main1.py  src/OTHER/picture.py src/DRIVER_FUNCTIONS/drive.py src/DRIVER_FUNCTIONS/LB.py src/INIT/init.py src/OTHER/testcode.py src/PID_FORWARD/PID.py src/PID_FORWARD/NEWPID.py src/PID_TURN/PIDTURN.py src/AUTO/autonomous.py src/AUTO/autoselect.py> src/main.py

# Output message
echo "Bundling complete: main.py has been generated and it's ready to upload!"
