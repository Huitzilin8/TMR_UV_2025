import subprocess

command = ["ls", "-l", "/tmp"]
try:
    # It returns the exit status of the command (0 usually means success).
    print "Executing command: {}".format(" ".join(command))
    return_code = subprocess.call(command)

    # --- Check the return code (optional) ---
    if return_code == 0:
        print "Command executed successfully."
    else:
        print "Command failed with return code: {}".format(return_code)
