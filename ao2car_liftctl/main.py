import subprocess
import sys
import signal
import os
import select

def main():
    process = subprocess.Popen(
        ["python3", "app.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    try:
        while True:
            # 使用 select 监控 stdout 和 stderr
            readable, _, _ = select.select([process.stdout, process.stderr], [], [], 0.1)
            for stream in readable:
                line = stream.readline()
                if line:
                    if stream == process.stdout:
                        print("app.py stdout:", line.strip())
                    elif stream == process.stderr:
                        print("app.py stderr:", line.strip())
                        if "DTLS_TRANSPORT_CLOSED" in line:
                            print("DTLS_TRANSPORT_CLOSED error detected, sending signal...")
                            os.kill(process.pid, signal.SIGUSR1)
            if process.poll() is not None:
                break
    except KeyboardInterrupt:
        print("Interrupted, terminating app.py...")
        process.terminate()
    finally:
        return process.poll()

if __name__ == "__main__":
    sys.exit(main())