#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
set -a; . ./.env; set +a

# 기존 파일 삭제 (File exists 에러 방지)
rm -f "$PWD/mosquitto/config/password_file" "$PWD/mosquitto/config/password_file.tmp"

docker run --rm -e MQTT_USER -e MQTT_PASS -e SENSOR_PASS -e GATEWAY_PASS -e MON_PASS \
  -v "$PWD/mosquitto/config:/mosquitto/config" eclipse-mosquitto:2 sh -c '
  set -e
  f=/mosquitto/config/password_file
  mosquitto_passwd -c -b "$f" "$MQTT_USER" "$MQTT_PASS"
  mosquitto_passwd -b "$f" sensor "$SENSOR_PASS"
  mosquitto_passwd -b "$f" edgeai "$MQTT_PASS"
  mosquitto_passwd -b "$f" gateway "$GATEWAY_PASS"
  mosquitto_passwd -b "$f" monitor "$MON_PASS"
  chmod 0644 "$f"
'
echo "[성공] 202345021 포함 5개 계정 암호 파일 생성 완료"
