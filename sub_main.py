import subprocess

# --- Define the command ---
# It's best to provide the command and its arguments as a list.
# For example, to run 'ls -l /tmp'
command = ["ls", "-l", "/tmp"]
# Or a simpler command like 'mkdir new_directory'
# command = ["mkdir", "new_directory"]

try:
    # --- Execute the command ---
    # subprocess.call() will execute the command and wait for it to complete.
    # It returns the exit status of the command (0 usually means success).
    print "Executing command: {}".format(" ".join(command))
    return_code = subprocess.call(command)

    # --- Check the return code (optional) ---
    if return_code == 0:
        print "Command executed successfully."
    else:
        print "Command failed with return code: {}".format(return_code)
