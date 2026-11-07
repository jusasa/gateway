import argparse
import csv
import json
import os
import time
import paho.mqtt.client as mqtt

# CLI 인자 정의
parser = argparse.ArgumentParser(description="PHM 진동 데이터셋 CSV 수집기")
parser.add_argument("--label", type=str, required=True, help="데이터 라벨 (예: normal, unbal1g, loose, off)")
parser.add_argument("--speed", type=int, default=2, help="선풍기 바람 세기 (1, 2, 3 또는 OFF=0)")
parser.add_argument("--duration", type=int, default=60, help="수집 시간(초, 기본 60초)")
parser.add_argument("--output", type=str, default="vibration_dataset.csv", help="저장할 CSV 파일명")
args = parser.parse_args()

CSV_FILE = args.output
FIELDS = [
    "timestamp", "seq", "label", "fan_speed",
    "acc_rms_g", "crest_factor", "kurtosis", "rpm_est",
    "band_1x", "band_harm", "vel_rms_mms"
]

# 기존 파일이 없으면 헤더 생성, 있으면 뒤에 이어 쓰기(append)
file_exists = os.path.exists(CSV_FILE)
csv_fp = open(CSV_FILE, mode="a", newline="", encoding="utf-8")
writer = csv.DictWriter(csv_fp, fieldnames=FIELDS)

if not file_exists:
    writer.writeheader()
    csv_fp.flush()

sample_count = 0
start_time = time.time()

def on_connect(client, userdata, flags, rc):
    print(f"[*] 브로커 접속 완료 (rc={rc})")
    print(f"[*] 수집 시작: 라벨='{args.label}', 바람세기={args.speed}단, 목표시간={args.duration}초")
    client.subscribe("factory/machine01/diagnostics", qos=1)

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
            "vel_rms_mms": data.get("vel_rms_mms", 0.0)
        }
        writer.writerow(row)
        csv_fp.flush()
        sample_count += 1
        elapsed = int(time.time() - start_time)
        print(f"[{sample_count:02d}개 수집 | {elapsed}s/{args.duration}s] "
              f"RMS={row['acc_rms_g']:.4f}g, CF={row['crest_factor']:.2f}, "
              f"Kurt={row['kurtosis']:.2f}, RPM={row['rpm_est']:.1f}")
    except Exception as e:
        print(f"[!] 데이터 처리 오류: {e}")

client = mqtt.Client(client_id=f"csv-logger-{os.getpid()}")
client.username_pw_set("202345021", "1234")
client.on_connect = on_connect
client.on_message = on_message

client.connect("127.0.0.1", 1883, 60)
client.loop_start()

try:
    while time.time() - start_time < args.duration:
        time.sleep(0.5)
finally:
    client.loop_stop()
    client.disconnect()
    csv_fp.close()
    print(f"\n[완료] '{args.label}' 상태 데이터 {sample_count}개 수집 완료 -> '{CSV_FILE}' 누적 완료.\n")
