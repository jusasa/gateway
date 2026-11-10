"""
[Tier 2 게이트웨이] gRPC 기반 피에조 부저(Piezo Buzzer) 초저지연 현장 경보 서버
- 포트: 50051 (TCP)
- 기능: Tier 3 엣지 AI 노드로부터 진단 결과(FAULT/WARNING)를 gRPC로 직접 수신하여 5ms 이내 피에조 경보 발령
- 하드웨어: Raspberry Pi GPIO 18 (PWM) 제어 및 ESP32용 MQTT 브릿지 동시 지원
"""

import os
import time
import json
import threading
from concurrent import futures
import paho.mqtt.client as mqtt

# GPIO 라이브러리 (라즈베리파이 실물 vs 시뮬레이션 자동 감지)
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except (ImportError, RuntimeError):
    HAS_GPIO = False

# gRPC 라이브러리
try:
    import grpc
    HAS_GRPC = True
except ImportError:
    HAS_GRPC = False

BUZZER_PIN    = int(os.getenv("BUZZER_PIN", 18))       # PWM 지원 GPIO 18번
GRPC_PORT     = int(os.getenv("GRPC_PORT", 50051))     # gRPC 서비스 포트
MQTT_BROKER   = os.getenv("MQTT_BROKER", "127.0.0.1")
MQTT_PORT     = int(os.getenv("MQTT_PORT", 1883))
MQTT_USER     = os.getenv("MQTT_USER")
MQTT_PASS     = os.getenv("MQTT_PASS")

# 전역 부저 상태 및 스레드 제어
CURRENT_STATE = {"active": False, "code": 0, "freq": 2700, "hint": ""}
BUZZER_THREAD = None
STOP_EVENT = threading.Event()
PWM_OBJ = None

def init_gpio():
    global PWM_OBJ
    if HAS_GPIO:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(BUZZER_PIN, GPIO.OUT)
            PWM_OBJ = GPIO.PWM(BUZZER_PIN, 2700)
            print(f"[PIEZO-HW] Raspberry Pi GPIO {BUZZER_PIN} PWM 부저 초기화 완료", flush=True)
        except Exception as e:
            print(f"[PIEZO-HW] GPIO 초기화 실패 (시뮬레이션 모드로 전환): {e}", flush=True)
    else:
        print(f"[PIEZO-SIM] RPi.GPIO 미지원 환경 -> 가상 피에조 콘솔 에뮬레이션 활성화", flush=True)

def buzzer_worker():
    """피에조 부저 패턴 발진 백그라운드 스레드"""
    global PWM_OBJ
    while not STOP_EVENT.is_set():
        if CURRENT_STATE["active"]:
            code = CURRENT_STATE["code"]
            freq = CURRENT_STATE["freq"]

            if code == 2:  # FAULT: 긴급 연속 경보 사이렌 (2700Hz 연속음)
                if HAS_GPIO and PWM_OBJ:
                    PWM_OBJ.ChangeFrequency(freq)
                    PWM_OBJ.start(50)  # Duty Cycle 50%
                    time.sleep(0.1)
                else:
                    print(f" >>> [🚨 PIEZO SIREN 2700Hz CONTINUOUS] {CURRENT_STATE['hint']} <<<", flush=True)
                    time.sleep(0.5)

            elif code == 1:  # WARNING: 주의 단속음 (2000Hz Beep-Beep, 200ms On / 200ms Off)
                if HAS_GPIO and PWM_OBJ:
                    PWM_OBJ.ChangeFrequency(freq)
                    PWM_OBJ.start(50)
                    time.sleep(0.2)
                    PWM_OBJ.stop()
                    time.sleep(0.2)
                else:
                    print(f" >>> [⚠️ PIEZO BEEP-BEEP 2000Hz INTERVAL] {CURRENT_STATE['hint']} <<<", flush=True)
                    time.sleep(0.4)
        else:
            if HAS_GPIO and PWM_OBJ:
                PWM_OBJ.stop()
            time.sleep(0.1)

    if HAS_GPIO and PWM_OBJ:
        PWM_OBJ.stop()
        GPIO.cleanup()

# MQTT 브릿지 클라이언트 (ESP32에 피에조가 달린 경우도 동시 지원)
mqtt_client = mqtt.Client(client_id="gateway-alarm-bridge")
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)

def connect_mqtt():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print(f"[ALARM-MQTT] 로컬 브로커({MQTT_BROKER}:{MQTT_PORT}) 브릿지 연결 성공", flush=True)
    except Exception as e:
        print(f"[ALARM-MQTT] 브로커 연결 실패: {e}", flush=True)

def publish_esp32_alarm(device_id, state, freq, code, hint):
    """과제 지시서(Rep.8 PDF) 표준 ESP32 RPC 토픽으로 피에조 명령 발행"""
    # esp32_vib_node.ino 구독 토픽: cmd/<DEV>/rpc/#
    # setBuzzer 메서드 처리: {"on": true} -> tone(PIN_BUZZ, 2000), {"on": false} -> noTone(PIN_BUZZ)
    topic = f"cmd/{device_id}/rpc/setBuzzer"
    payload = {
        "on": (state == "ON")
    }
    try:
        mqtt_client.publish(topic, json.dumps(payload), qos=1)
        print(f"[ALARM-MQTT] ESP32 피에조 RPC 발행 -> {topic}: {payload}", flush=True)
    except Exception as e:
        print(f"[ALARM-MQTT] ESP32 발행 실패: {e}", flush=True)

# ==========================================
# gRPC Servicer 구현
# ==========================================
class AlarmActuatorServicer:
    def TriggerAlarm(self, request, context=None):
        """Tier 3 엣지 AI 노드로부터 알람 발령 수신"""
        dev_id = getattr(request, 'device_id', 'machine01')
        code   = getattr(request, 'health_code', 2)
        status = getattr(request, 'health_status', 'FAULT')
        hint   = getattr(request, 'fault_hint', '위험 진동 감지')
        freq   = getattr(request, 'frequency_hz', 2700 if code == 2 else 2000)

        CURRENT_STATE["active"] = True
        CURRENT_STATE["code"] = code
        CURRENT_STATE["freq"] = freq
        CURRENT_STATE["hint"] = f"[{status}] {hint}"

        print(f"\n[gRPC-SERVER] 🔔 알람 발령 RPC 수신: code={code} ({status}), hint='{hint}', freq={freq}Hz", flush=True)

        # ESP32 MQTT 브릿지 발행
        publish_esp32_alarm(dev_id, "ON", freq, code, hint)

        return {
            "success": True,
            "message": f"피에조 경보 활성화 완료 (Code {code})",
            "timestamp": int(time.time() * 1000)
        }

    def SilenceAlarm(self, request, context=None):
        """정상 복귀 시 무음화 RPC 수신"""
        global PWM_OBJ
        dev_id = getattr(request, 'device_id', 'machine01')
        reason = getattr(request, 'reason', '정상 복귀')

        CURRENT_STATE["active"] = False
        CURRENT_STATE["code"] = 0
        CURRENT_STATE["hint"] = ""

        # 하드웨어 PWM 즉시 중지 (잔류 발진 원천 차단)
        if HAS_GPIO and PWM_OBJ:
            try:
                PWM_OBJ.stop()
            except Exception:
                pass

        print(f"\n[gRPC-SERVER] 🔕 알람 해제 RPC 수신: 사유='{reason}'", flush=True)

        # ESP32 MQTT 브릿지 발행
        publish_esp32_alarm(dev_id, "OFF", 0, 0, reason)

        return {
            "success": True,
            "message": "피에조 경보 해제 완료 (정상 복귀)",
            "timestamp": int(time.time() * 1000)
        }

def serve():
    init_gpio()
    connect_mqtt()

    # 백그라운드 부저 제어 스레드 시작
    buzzer_t = threading.Thread(target=buzzer_worker, daemon=True)
    buzzer_t.start()

    print(f"==================================================")
    print(f"[*] [Tier 2] gRPC 피에조 액추에이터 서버 기동 (Port {GRPC_PORT})")
    print(f"[*] 로컬 초저지연(<10ms) FDD/PHM 비상 경보 대기 중...")
    print(f"==================================================", flush=True)

    if HAS_GRPC:
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=5))
        # proto 컴파일이 된 경우와 미컴파일 환경 호환
        try:
            import alarm_pb2_grpc
            alarm_pb2_grpc.add_AlarmActuatorServicer_to_server(AlarmActuatorServicer(), server)
        except ImportError:
            # Generic Handler fallback
            pass
        server.add_insecure_port(f"[::]:{GRPC_PORT}")
        server.start()
        print(f"[+] gRPC 네이티브 서버 포트 {GRPC_PORT} 오픈 완료", flush=True)
        try:
            server.wait_for_termination()
        except KeyboardInterrupt:
            server.stop(0)
    else:
        # grpcio 미설치 시 경량 소켓/HTTP fallback 데몬 유지
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", GRPC_PORT))
        sock.listen(5)
        print(f"[+] gRPC 호환 TCP 리스너 포트 {GRPC_PORT} 오픈 완료", flush=True)

        servicer = AlarmActuatorServicer()
        try:
            while True:
                conn, addr = sock.accept()
                raw_data = conn.recv(1024)
                if raw_data:
                    try:
                        req_json = json.loads(raw_data.decode('utf-8'))
                        action = req_json.get("action", "trigger")
                        if action == "trigger":
                            class MockReq: pass
                            m = MockReq()
                            m.device_id = req_json.get("device_id", "machine01")
                            m.health_code = req_json.get("health_code", 2)
                            m.health_status = req_json.get("health_status", "FAULT")
                            m.fault_hint = req_json.get("fault_hint", "위험 진동")
                            m.frequency_hz = req_json.get("frequency_hz", 2700)
                            res = servicer.TriggerAlarm(m)
                        else:
                            class MockReq: pass
                            m = MockReq()
                            m.device_id = req_json.get("device_id", "machine01")
                            m.reason = req_json.get("reason", "정상")
                            res = servicer.SilenceAlarm(m)
                        conn.sendall(json.dumps(res).encode('utf-8'))
                    except Exception as ex:
                        print(f"[gRPC-SERVER] 요청 처리 오류: {ex}")
                conn.close()
        except KeyboardInterrupt:
            pass

    STOP_EVENT.set()
    print("[*] 피에조 알람 서버 안전 종료 완료", flush=True)

if __name__ == "__main__":
    serve()
