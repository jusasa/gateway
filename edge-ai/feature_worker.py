import json
import os
import time
import numpy as np
import paho.mqtt.client as mqtt
from scipy.stats import kurtosis

# 브로커 및 선풍기 물리 규격 설정
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
MQTT_USER   = os.getenv("MQTT_USER", "202345021")
MQTT_PASS   = os.getenv("MQTT_PASS", "1234")
FAN_SPEED   = int(os.getenv("FAN_SPEED", 1))
BLADE_COUNT = int(os.getenv("BLADE_COUNT", 5))  # 실물 선풍기 날개 수 (Z = 5)
SHAFT_SPEED_RPM = float(os.getenv("SHAFT_SPEED_RPM", 0.0))  # 0.0: 자동 하모닉 탐색, >0: 수동 오버라이드 (Step 2 규격)

def extract_features(raw_samples, fs=1000.0):
    """
    ESP32 원시 파형에서 ML 8대 특징량 및 5날개 통과 주파수(BPF) 고속 계산
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

    # 14Hz ~ 35Hz (840 ~ 2100 RPM) 회전 1x 대역 타깃팅 (바닥 진동 12.7Hz 간섭 배제)
    valid_idx = np.where((freqs >= 14.0) & (freqs <= 35.0))[0]

    VIB_RUNNING_THRESHOLD_G = 0.050
    PEAK_AMPLITUDE_MIN_G   = 0.008

    f_1x = 0.0
    band_1x = 0.0
    rpm_est = 0.0
    band_harm = 0.0
    f_bp_calc = 0.0
    f_bp_meas = 0.0
    band_bp = 0.0

    if rms_val >= VIB_RUNNING_THRESHOLD_G:
        # A. 수동 회전수 강제 오버라이드 모드 (ThingsBoard shaftSpeedRpm 하향 시)
        if SHAFT_SPEED_RPM > 0.0:
            f_1x = float(SHAFT_SPEED_RPM / 60.0)
            rpm_est = float(SHAFT_SPEED_RPM)
            w_1x = np.where((freqs >= f_1x - 1.5) & (freqs <= f_1x + 1.5))[0]
            band_1x = float(np.max(fft_vals[w_1x])) if len(w_1x) > 0 else 0.0

            # Step 2: 5날개 통과 주파수(BPF = Z * f_1x) 이론 계산 및 주변 피크 탐색
            f_bp_calc = float(BLADE_COUNT * f_1x)
            bp_window = np.where((freqs >= f_bp_calc - 2.5) & (freqs <= f_bp_calc + 2.5))[0]
            if len(bp_window) > 0:
                bp_idx = bp_window[np.argmax(fft_vals[bp_window])]
                f_bp_meas = float(freqs[bp_idx])
                band_bp = float(fft_vals[bp_idx])

        # B. [방법 1 구현] BPF 역추적 기반 1x 자동 확정 (BPF-first Inverted Tracking)
        # 16Hz 구조 공진 대역을 원천 우회하기 위해, 고주파 BPF 대역(80~140Hz)에서 
        # 공기역학적 날개 통과 피크(f_bpf)를 먼저 탐색한 후 f_1x = f_bpf / Z 로 역산
        else:
            # 80Hz ~ 140Hz BPF 고주파 탐색 윈도우 (Z=5 기준 16Hz ~ 28Hz 회전 대응)
            bpf_window = np.where((freqs >= 80.0) & (freqs <= 140.0))[0]
            bpf_found = False

            if len(bpf_window) > 0:
                # 80~140Hz 대역 내 국부 극대점 탐색
                bpf_cands = []
                for b_idx in bpf_window:
                    if 0 < b_idx < len(fft_vals) - 1:
                        if fft_vals[b_idx] > fft_vals[b_idx - 1] and fft_vals[b_idx] > fft_vals[b_idx + 1]:
                            bpf_cands.append(b_idx)
                
                # 극대점이 없으면 해당 윈도우 내 최대값 사용
                if not bpf_cands:
                    bpf_top_idx = bpf_window[np.argmax(fft_vals[bpf_window])]
                else:
                    bpf_top_idx = max(bpf_cands, key=lambda i: fft_vals[i])

                f_bpf_candidate = float(freqs[bpf_top_idx])
                band_bpf_candidate = float(fft_vals[bpf_top_idx])

                # BPF 피크가 유의미한 진폭을 가지면 BPF 역추적 적용
                if band_bpf_candidate >= PEAK_AMPLITUDE_MIN_G:
                    f_bp_meas = f_bpf_candidate
                    band_bp = band_bpf_candidate
                    f_bp_calc = f_bpf_candidate

                    # 1번 역산 공식: f_1x = f_bpf / BLADE_COUNT
                    f_1x = float(f_bp_meas / max(1, BLADE_COUNT))
                    rpm_est = float(f_1x * 60.0)

                    # 역산된 f_1x 주변(±1.5Hz)에서 실제 1x 진동 진폭 추출
                    w_1x = np.where((freqs >= f_1x - 1.5) & (freqs <= f_1x + 1.5))[0]
                    band_1x = float(np.max(fft_vals[w_1x])) if len(w_1x) > 0 else 0.0
                    bpf_found = True

            # C. Fallback: BPF 피크가 명확하지 않은 경우 하모닉 체인 탐색 수행
            if not bpf_found and len(valid_idx) > 0:
                cand_indices = []
                for idx in valid_idx:
                    if 0 < idx < len(fft_vals) - 1:
                        if fft_vals[idx] > fft_vals[idx - 1] and fft_vals[idx] > fft_vals[idx + 1]:
                            if fft_vals[idx] >= PEAK_AMPLITUDE_MIN_G:
                                cand_indices.append(idx)
                
                if not cand_indices:
                    top_k = min(3, len(valid_idx))
                    cand_indices = list(valid_idx[np.argsort(fft_vals[valid_idx])[-top_k:]])
                else:
                    cand_indices = sorted(cand_indices, key=lambda i: fft_vals[i], reverse=True)[:5]

                best_score = -1.0
                best_idx = cand_indices[0]

                for c_idx in cand_indices:
                    f_cand = freqs[c_idx]
                    a_1x = fft_vals[c_idx]

                    f_2x = 2.0 * f_cand
                    w_2x = np.where((freqs >= f_2x - 1.5) & (freqs <= f_2x + 1.5))[0]
                    a_2x = float(np.max(fft_vals[w_2x])) if len(w_2x) > 0 else 0.0

                    f_bpf = float(BLADE_COUNT) * f_cand
                    w_bpf = np.where((freqs >= f_bpf - 2.5) & (freqs <= f_bpf + 2.5))[0]
                    a_bpf = float(np.max(fft_vals[w_bpf])) if len(w_bpf) > 0 else 0.0

                    h_score = a_1x + (0.5 * a_2x) + (0.8 * a_bpf)
                    if h_score > best_score:
                        best_score = h_score
                        best_idx = c_idx

                f_1x = float(freqs[best_idx])
                band_1x = float(fft_vals[best_idx])

                if band_1x >= PEAK_AMPLITUDE_MIN_G:
                    rpm_est = float(f_1x * 60.0)
                    f_bp_calc = float(BLADE_COUNT * f_1x)
                    bp_window = np.where((freqs >= f_bp_calc - 2.5) & (freqs <= f_bp_calc + 2.5))[0]
                    if len(bp_window) > 0:
                        bp_idx = bp_window[np.argmax(fft_vals[bp_window])]
                        f_bp_meas = float(freqs[bp_idx])
                        band_bp = float(fft_vals[bp_idx])
                else:
                    rpm_est = 0.0
                    band_1x = 0.0

        # 고조파 대역 에너지 합 (40Hz ~ 200Hz - 5x BPF 포함 대역)
        band_harm = float(np.sum(fft_vals[(freqs >= 40.0) & (freqs <= 200.0)]))

    # 파생 상대 지표 (r1x, rh) 및 속도 RMS
    r1x = round(band_1x / rms_val, 3) if rms_val > 0 else 0.0
    rh = round(band_harm / rms_val, 3) if rms_val > 0 else 0.0
    vel_rms = round(rms_val * 9806.65 / (2 * np.pi * max(15.0, rpm_est / 60.0)) / 1000.0, 3) if rpm_est > 0 else 0.0

    return {
        # ML 8대 특징량 (model.joblib 호환 유지)
        "acc_rms_g": round(rms_val, 4),
        "crest_factor": round(crest_factor, 3),
        "kurtosis": round(kurt, 3),
        "rpm_est": round(rpm_est, 1),
        "band_1x": round(band_1x, 4),
        "band_harm": round(band_harm, 4),
        "r1x": r1x,
        "rh": rh,
        "vel_rms_mms": vel_rms,
        # Step 2 날개 통과 주파수 지표 (Z = 5)
        "blade_count": BLADE_COUNT,
        "f_1x": round(f_1x, 2),
        "f_bp_calc": round(f_bp_calc, 2),
        "f_bp_meas": round(f_bp_meas, 2),
        "band_bp": round(band_bp, 4)
    }

def on_connect(client, userdata, flags, rc):
    print(f"[GATEWAY-EXTRACTOR] 브로커 연결 성공 (rc={rc})", flush=True)
    # 원시 파형 및 ThingsBoard 설정 하향 토픽 구독
    client.subscribe([("raw/+/sensor", 1), ("config/+/attributes", 1)])
    print("[GATEWAY-EXTRACTOR] 구독 토픽 등록: raw/+/sensor, config/+/attributes", flush=True)

def on_message(client, userdata, msg):
    global FAN_SPEED, BLADE_COUNT, SHAFT_SPEED_RPM
    try:
        parts = msg.topic.split('/')
        
        # 1. ThingsBoard 설정 하향 (config/<dev>/attributes) 처리
        if parts[0] == "config":
            payload_str = msg.payload.decode('utf-8')
            cfg = json.loads(payload_str)
            if "fanSpeed" in cfg:
                FAN_SPEED = int(cfg["fanSpeed"])
            if "bladeCount" in cfg:
                BLADE_COUNT = int(cfg["bladeCount"])
            if "shaftSpeedRpm" in cfg:
                SHAFT_SPEED_RPM = float(cfg["shaftSpeedRpm"])
            print(f"[GATEWAY-CFG] 속성 하향 갱신: fanSpeed={FAN_SPEED}, bladeCount={BLADE_COUNT}, shaftSpeedRpm={SHAFT_SPEED_RPM}", flush=True)
            return

        # 2. ESP32 원시 파형 수신 및 특징 추출
        device_name = parts[1]
        data = json.loads(msg.payload.decode('utf-8'))
        raw_samples = data.get("vib_raw", [])
        fs_meas = data.get("fs_meas", 1000.0)
        seq = data.get("seq", 0)
        ts = int(data.get("ts", time.time() * 1000))

        feats = extract_features(raw_samples, fs=fs_meas)
        if not feats:
            return

        # 3. 엣지 AI 파이로 중계할 정제 텔레메트리 패키징
        features_payload = {
            "seq": int(seq),
            "ts": ts,
            "fs_meas": round(float(fs_meas), 1),
            "fan_speed": FAN_SPEED,
            **feats
        }

        # 4. RPi #2 (Edge AI) 전용 토픽 발행 (95.8% 이상 대역폭 압축)
        out_topic = f"factory/{device_name}/features"
        client.publish(out_topic, json.dumps(features_payload), qos=1)

        print(
            f"[EXTRACT] seq={seq} | s{FAN_SPEED} | RMS={feats['acc_rms_g']}g | "
            f"1x={feats['f_1x']}Hz ({feats['rpm_est']} RPM) | "
            f"BPF(5x)={feats['f_bp_meas']}Hz (계산={feats['f_bp_calc']}Hz, 진폭={feats['band_bp']}g)",
            flush=True
        )

    except Exception as e:
        print(f"[ERROR] 특징 추출 실패: {e}", flush=True)

if __name__ == "__main__":
    client = mqtt.Client(client_id="gateway-feature-extractor")
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(1, 30)

    print(f"[GATEWAY-EXTRACTOR] {MQTT_BROKER}:{MQTT_PORT} 접속 시도 중...", flush=True)
    while True:
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            break
        except Exception as e:
            print(f"[GATEWAY-EXTRACTOR] 접속 실패 ({e}), 2초 후 재시도...", flush=True)
            time.sleep(2)

    client.loop_forever()
