import json
import os
import time
import numpy as np
import paho.mqtt.client as mqtt
from scipy.stats import kurtosis

# 브로커 설정 (게이트웨이 로컬 Mosquitto 컨테이너)
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
MQTT_USER   = os.getenv("MQTT_USER", "202345021")
MQTT_PASS   = os.getenv("MQTT_PASS", "1234")
FAN_SPEED   = int(os.getenv("FAN_SPEED", 1))

def extract_features(raw_samples, fs=1000.0):
    """
    ESP32 원시 파형에서 ML 모델 추론에 필요한 8대 특징량 고속 계산
    """
    x = np.array(raw_samples, dtype=np.float64)
    N = len(x)
    if N == 0:
        return None

    # 1. DC 바이어스 보정 (중력가속도 오프셋 제거)
    x_ac = x - np.mean(x)

    # 2. RMS 계산
    rms = np.sqrt(np.mean(x_ac ** 2))
    rms_val = float(rms) if rms > 1e-9 else 1e-9

    # 3. Peak, Crest Factor, Kurtosis
    peak = float(np.max(np.abs(x_ac)))
    crest_factor = float(peak / rms_val)
    kurt = float(kurtosis(x_ac, fisher=False))

    # 4. Hanning 윈도우 기반 FFT (스펙트럼 누설 억제 및 진폭 복원)
    win = np.hanning(N)
    fft_vals = np.abs(np.fft.rfft(x_ac * win)) * (4.0 / N)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)

    # 14Hz ~ 35Hz (840 ~ 2100 RPM) 회전 대역 타깃팅 (바닥 진동 12.7Hz 차단)
    valid_idx = np.where((freqs >= 14.0) & (freqs <= 35.0))[0]

    VIB_RUNNING_THRESHOLD_G = 0.050
    PEAK_AMPLITUDE_MIN_G   = 0.008

    if len(valid_idx) > 0 and rms_val >= VIB_RUNNING_THRESHOLD_G:
        peak_idx = valid_idx[np.argmax(fft_vals[valid_idx])]
        f_1x = freqs[peak_idx]
        band_1x = float(fft_vals[peak_idx])

        # 피크 진폭 유효성 검증
        if band_1x >= PEAK_AMPLITUDE_MIN_G:
            rpm_est = float(f_1x * 60.0)
        else:
            rpm_est = 0.0
            band_1x = 0.0

        # 고조파 대역 에너지 합
        band_harm = float(np.sum(fft_vals[(freqs >= 40.0) & (freqs <= 200.0)]))
    else:
        rpm_est = 0.0
        band_1x = 0.0
        band_harm = 0.0

    # 파생 상대 지표 (r1x, rh) 및 속도 RMS
    r1x = round(band_1x / rms_val, 3) if rms_val > 0 else 0.0
    rh = round(band_harm / rms_val, 3) if rms_val > 0 else 0.0
    vel_rms = round(rms_val * 9806.65 / (2 * np.pi * max(15.0, rpm_est / 60.0)) / 1000.0, 3) if rpm_est > 0 else 0.0

    return {
        "acc_rms_g": round(rms_val, 4),
        "crest_factor": round(crest_factor, 3),
        "kurtosis": round(kurt, 3),
        "rpm_est": round(rpm_est, 1),
        "band_1x": round(band_1x, 4),
        "band_harm": round(band_harm, 4),
        "r1x": r1x,
        "rh": rh,
        "vel_rms_mms": vel_rms
    }

def on_connect(client, userdata, flags, rc):
    print(f"[GATEWAY-EXTRACTOR] 브로커 연결 성공 (rc={rc})")
    client.subscribe("raw/+/sensor", qos=1)
    print("[GATEWAY-EXTRACTOR] 구독 토픽: raw/+/sensor")

def on_message(client, userdata, msg):
    try:
        parts = msg.topic.split('/')
        device_name = parts[1]

        data = json.loads(msg.payload.decode('utf-8'))
        raw_samples = data.get("vib_raw", [])
        fs_meas = data.get("fs_meas", 1000.0)
        seq = data.get("seq", 0)
        ts = int(data.get("ts", time.time() * 1000))

        # 1. 고속 특징 추출
        feats = extract_features(raw_samples, fs=fs_meas)
        if not feats:
            return

        # 2. 엣지 AI 추론용 입력 페이로드 패키징 (순수 수치 데이터만 포함)
        features_payload = {
            "seq": int(seq),
            "ts": ts,
            "fs_meas": round(float(fs_meas), 1),
            "fan_speed": FAN_SPEED,
            **feats
        }

        # 3. 엣지 AI 파이가 구독할 전용 토픽으로 발행
        out_topic = f"factory/{device_name}/features"
        client.publish(out_topic, json.dumps(features_payload), qos=1)

        print(f"[EXTRACT] seq={seq} -> {out_topic} | RMS={feats['acc_rms_g']}g | 1x={feats['band_1x']}g | RPM={feats['rpm_est']}")

    except Exception as e:
        print(f"[ERROR] 특징 추출 실패: {e}")

if __name__ == "__main__":
    client = mqtt.Client(client_id="gateway-feature-extractor")
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[GATEWAY-EXTRACTOR] {MQTT_BROKER}:{MQTT_PORT} 접속 시도 중...")
    while True:
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            break
        except Exception:
            time.sleep(2)

    client.loop_forever()
