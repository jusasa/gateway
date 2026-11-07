import argparse
import csv
import json
import os
import time
import paho.mqtt.client as mqtt

# CLI 인자 정의
parser = argparse.ArgumentParser(description="PHM 선풍기 진동 데이터셋 CSV 수집기 (Rep. 8)")
parser.add_argument("--label", type=str, required=True, help="데이터 라벨 (예: normal, unbal1g, loose, off)")
parser.add_argument("--speed", type=int, default=2, help="선풍기 바람 세기 (1, 2, 3 또는 OFF=0)")
parser.add_argument("--duration", type=int, default=60, help="수집 시간(초, 기본 60초)")
parser.add_argument("--host", type=str, default=os.getenv("MQTT_HOST", "127.0.0.1"), help="Mosquitto 브로커 IP")
parser.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", 1883)), help="Mosquitto 포트")
parser.add_argument("--user", type=str, default=os.getenv("MQTT_USER", "202345021"), help="MQTT 사용자")
parser.add_argument("--password", type=str, default=os.getenv("MQTT_PASS", "1234"), help="MQTT 비밀번호")
parser.add_argument("--output", type=str, default="vibration_dataset.csv", help="저장할 CSV 파일명")
args = parser.parse_args()

CSV_FILE = args.output
FIELDS = [
    "timestamp", "seq", "label", "fan_speed",
    "acc_rms_g", "crest_factor", "kurtosis", "rpm_est",
    "band_1x", "band_harm", "r1x", "rh", "vel_rms_mms",
    "f_1x", "f_bp_calc", "f_bp_meas", "band_bp",
    "anomaly_score", "health_code", "health_status"
]

file_exists = os.path.exists(CSV_FILE)
csv_fp = open(CSV_FILE, mode="a", newline="", encoding="utf-8")
writer = csv.DictWriter(csv_fp, fieldnames=FIELDS)

if not file_exists:
    writer.writeheader()
    csv_fp.flush()

sample_count = 0
start_time = time.time()

def on_connect(client, userdata, flags, rc):
    print(f"[*] 브로커({args.host}:{args.port}) 접속 완료 (rc={rc})")
    print(f"[*] 데이터 수집 시작: 라벨='{args.label}', 바람세기={args.speed}단, 목표시간={args.duration}초")
    client.subscribe("factory/+/diagnostics", qos=1)

def on_message(client, userdata, msg):
    global sample_count
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        row = {
            "timestamp": int(time.time()),
            "seq": data.get("seq", 0),
            "label": args.label,
            "fan_speed": args.speed,
            "acc_rms_g": data.get("acc_rms_g", 0.0),
            "crest_factor": data.get("crest_factor", 0.0),
            "kurtosis": data.get("kurtosis", 0.0),
            "rpm_est": data.get("rpm_est", 0.0),
            "band_1x": data.get("band_1x", 0.0),
            "band_harm": data.get("band_harm", 0.0),
            "r1x": data.get("r1x", 0.0),
            "rh": data.get("rh", 0.0),
            "vel_rms_mms": data.get("vel_rms_mms", 0.0),
            "f_1x": data.get("f_1x", 0.0),
            "f_bp_calc": data.get("f_bp_calc", 0.0),
            "f_bp_meas": data.get("f_bp_meas", 0.0),
            "band_bp": data.get("band_bp", 0.0),
            "anomaly_score": data.get("anomaly_score", 0.0),
            "health_code": data.get("health_code", 0),
            "health_status": data.get("health_status", "NORMAL")
        }
        writer.writerow(row)
        csv_fp.flush()
        sample_count += 1
        elapsed = int(time.time() - start_time)
        print(f"[{sample_count:02d}개 수집 | {elapsed}s/{args.duration}s] "
              f"RMS={row['acc_rms_g']:.4f}g, 1x={row['f_1x']}Hz, "
              f"BPF={row['f_bp_meas']}Hz, Status={row['health_status']}")
    except Exception as e:
        print(f"[!] 데이터 처리 오류: {e}")

client = mqtt.Client(client_id=f"csv-logger-{os.getpid()}")
client.username_pw_set(args.user, args.password)
client.on_connect = on_connect
client.on_message = on_message

client.connect(args.host, args.port, 60)
client.loop_start()

try:
    while time.time() - start_time < args.duration:
        time.sleep(0.5)
finally:
    client.loop_stop()
    client.disconnect()
    csv_fp.close()
    print(f"\n[완료] '{args.label}' 상태 데이터 {sample_count}개 수집 완료 -> '{CSV_FILE}' 누적 저장.\n")
