import zmq, time, sys
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://localhost:5580")
time.sleep(0.5)
print("Keyboard publisher ready on :5580. Keys: k=start/stop loop, i=init pose, p=pause/resume, t <text>=prompt", flush=True)
while True:
    try:
        key = input()
    except EOFError:
        time.sleep(0.5); continue
    if key.startswith("t "):
        pub.send_string("prompt:" + key[2:]); print("Sent prompt: " + key[2:], flush=True)
    else:
        pub.send_string(key); print("Sent: " + repr(key), flush=True)
