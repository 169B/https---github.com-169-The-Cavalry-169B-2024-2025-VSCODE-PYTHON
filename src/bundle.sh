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
                                         
                                         
                                         
                                     
   169B
EOF

# 2-second intentional delay
sleep 0.5

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

# Delete main.py if it exists
if [ -f src/main.py ]; then
    rm src/main.py
    echo "Deleted existing main.py"
fi

# Combine all the files into the final main.py
files=(src/AUTO/autoselect.py src/MAIN_GB/main1.py src/OTHER/picture.py src/DRIVER_FUNCTIONS/drive.py src/DRIVER_FUNCTIONS/LB.py src/INIT/init.py src/OTHER/testcode.py src/PID_FORWARD/PID.py src/PID_FORWARD/NEWPID.py src/PID_TURN/PIDTURN.py src/AUTO/autonomous.py)
total_files=${#files[@]}
counter=0

for file in "${files[@]}"; do
    cat "$file" >> src/main.py
    counter=$((counter + 1))
    echo "Bundling progress: $counter/$total_files files"
    sleep 0.1  # Add a delay of 0.5 seconds for each file progress update
done

# Increment compile count
compile_count_file="/Users/jaybot/Documents/vex-vscode-projects/VexVSCodePython/src/compile_count.txt"
if [ ! -f "$compile_count_file" ]; then
    echo 0 > "$compile_count_file"
fi
compile_count=$(<"$compile_count_file")
compile_count=$((compile_count + 1))
echo "$compile_count" > "$compile_count_file"

# Output message
echo "Bundling complete: main.py has been generated and it's ready to upload!"


# Close the terminal
sleep 1
cat << "EOF"
 ████   ████████   ████████  ███████████ 
░░███  ███░░░░███ ███░░░░███░░███░░░░░███
 ░███ ░███   ░░░ ░███   ░███ ░███    ░███
 ░███ ░█████████ ░░█████████ ░██████████ 
 ░███ ░███░░░░███ ░░░░░░░███ ░███░░░░░███
 ░███ ░███   ░███ ███   ░███ ░███    ░███
 █████░░████████ ░░████████  ███████████ 
░░░░░  ░░░░░░░░   ░░░░░░░░  ░░░░░░░░░░░  
                                         
                                         
                                         
                                     
   169B
EOF
echo "You have compiled $compile_count times."
exit
