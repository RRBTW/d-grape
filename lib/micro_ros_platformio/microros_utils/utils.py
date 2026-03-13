import subprocess
import sys
import re
import tempfile
import os

def win_to_wsl(path):
    path = path.replace("\\", "/")
    result = subprocess.run(
        ["wsl", "wslpath", "-a", path],
        capture_output=True, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else path

def run_cmd(command, env=None):
    if sys.platform == "win32":
        def replace_path(m):
            return win_to_wsl(m.group(0))
        command = re.sub(r'\b[A-Za-z]:[/\\][\w/\\.:-]*', replace_path, command)

        # Remove Windows venv activation
        command = re.sub(r'\.\s+/mnt/\S+/activate\s*&&\s*', '', command)

        # Redirect build to WSL native filesystem
        command = re.sub(r'/mnt/[a-z]/[^\s]*/micro_ros_platformio/build',
                         '/home/roma/micro_ros_build', command)

        # Toolchain path in WSL
        toolchain_wsl = "/mnt/c/Users/Roma/.platformio/packages/toolchain-gccarmnoneeabi/bin"

        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False, newline='\n')
        tmp.write("#!/bin/bash\nset -e\n")
        tmp.write('export PATH="{}:$PATH"\n'.format(toolchain_wsl))
        tmp.write(command + "\n")
        tmp.close()

        wsl_tmp = subprocess.run(
            ["wsl", "wslpath", tmp.name.replace("\\", "/")],
            capture_output=True, text=True
        ).stdout.strip()

        result = subprocess.run(
            ["wsl", "bash", wsl_tmp],
            capture_output=True,
            env=env
        )
        os.unlink(tmp.name)
        return result

    return subprocess.run(command, capture_output=True, shell=True, env=env)
