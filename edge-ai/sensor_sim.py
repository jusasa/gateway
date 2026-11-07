import os
import json
import time
import argparse
import numpy as np
import paho.mqtt.client as mqtt

FS = 1000
N = 256
F_ROT = 30
F_LINE = 60

def make_window(mode, rng):
    t = np.arange(N) / FS
    amp, imp, cur_rms = 0.55, 0.0, 2.2
    if mode == "warning":
        amp, imp, cur_rms = 0.62, 0.45, 2.3
    elif mode == "fault":
        amp, imp, cur_rms = 0.75, 1.6, 2.6

    vib = amp * np.sin(2 * np.pi * F_ROT * t) + 0.05 * rng.randn(N)
    if imp > 0:
        period = int(FS / 87)
        for k in range(rng.randint(0, period), N, period):
            vib[k:k + 3] += imp * np.array([1.0, -0.6, 0.3])[:len(vib[k:k + 3])]
    cur = cur_rms * np.sqrt(2) * np.sin(2 * np.pi * F_LINE * t) + 0.03 * rng.randn(N)
    return vib, cur

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["normal", "warning", "fault"], default="normal")
    p.add_argument("--machine", default="machine01")
    p.add_argument("--count", type=int, default=20)
    p.add_argument("--interval", type=float, default=2.0)
    a = p.parse_args()

    client = mqtt.Client(client_id=f"sim_{a.machine}_{os.getpid()}")
    client.username_pw_set(os.getenv("MQTT_USER", "rep6iot"), os.getenv("MQTT_PASS", "rep6pass"))
    client.connect(os.getenv("MQTT_HOST", "mosquitto"), int(os.getenv("MQTT_PORT", "1883")), 60)
    client.loop_start()

    rng = np.random.RandomState()
    topic = f"raw/{a.machine}/sensor"
    for i in range(a.count):
        vib, cur = make_window(a.mode, rng)
        payload = {
            "fs": FS,
            "vib_raw": [round(float(v), 4) for v in vib],
            "cur_raw": [round(float(c), 4) for c in cur]
        }
        client.publish(topic, json.dumps(payload), qos=1).wait_for_publish()
        print(f"[SIM] {i + 1}/{a.count} {a.mode} -> {topic}")
        time.sleep(a.interval)

    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()
