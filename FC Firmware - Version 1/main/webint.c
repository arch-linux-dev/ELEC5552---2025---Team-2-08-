#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "externals.h"
#include "webint.h"


SemaphoreHandle_t vars_mutex = NULL; 

/* Webserver globals */
static httpd_handle_t server = NULL;
static int ws_client_sock = -1;

static const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ESP32 Flight Monitor</title>
<style>
body{font-family:Arial; padding:10px; max-width:900px;}
.row{margin-bottom:8px;}
label{display:inline-block; width:120px;}
input[type=range]{width:400px;}
.box{border:1px solid #ddd; padding:8px; margin-bottom:8px;}
</style>
</head>
<body>
<h2>ESP32 Flight Monitor</h2>

<div class="box">
  <div class="row">Receiver: <span id="rcv">[----]</span></div>
  <div class="row">Motors: <span id="mot">[----]</span></div>
  <div class="row">Gyro: Roll <span id="rRate">0</span> Pitch <span id="pRate">0</span> Yaw <span id="yRate">0</span></div>
  <div class="row">Accel: X <span id="ax">0</span> Y <span id="ay">0</span> Z <span id="az">0</span></div>
  <div class="row">Angles: Roll <span id="angR">0</span> Pitch <span id="angP">0</span></div>
  <div class="row">PID Outputs: Roll <span id="pidR">0</span> Pitch <span id="pidP">0</span> Yaw <span id="pidY">0</span></div>
</div>

<div class="box">
  <h3>Receiver sliders (ch0..ch3)</h3>
  <div class="row"><label>Roll (ch0):</label>
    <input id="rc0" type="range" min="1000" max="2000" step="1" value="1500" oninput="onRC(0,this.value)">
    <span id="rc0_val">1500</span></div>
  <div class="row"><label>Pitch (ch1):</label>
    <input id="rc1" type="range" min="1000" max="2000" step="1" value="1500" oninput="onRC(1,this.value)">
    <span id="rc1_val">1500</span></div>
  <div class="row"><label>Throttle (ch2):</label>
    <input id="rc2" type="range" min="1000" max="2000" step="1" value="1000" oninput="onRC(2,this.value)">
    <span id="rc2_val">1000</span></div>
  <div class="row"><label>Yaw (ch3):</label>
    <input id="rc3" type="range" min="1000" max="2000" step="1" value="1500" oninput="onRC(3,this.value)">
    <span id="rc3_val">1500</span></div>
</div>

<div class="box">
  <h3>Calibration values</h3>
  <div class="row"><label>RateCalRoll:</label><input id="RateCalibrationRoll" style="width:120px"><button onclick="setVar('RateCalibrationRoll')">Set</button></div>
  <div class="row"><label>RateCalPitch:</label><input id="RateCalibrationPitch" style="width:120px"><button onclick="setVar('RateCalibrationPitch')">Set</button></div>
  <div class="row"><label>RateCalYaw:</label><input id="RateCalibrationYaw" style="width:120px"><button onclick="setVar('RateCalibrationYaw')">Set</button></div>
  <div class="row"><label>AccXCal:</label><input id="AccXCalibration" style="width:120px"><button onclick="setVar('AccXCalibration')">Set</button></div>
  <div class="row"><label>AccYCal:</label><input id="AccYCalibration" style="width:120px"><button onclick="setVar('AccYCalibration')">Set</button></div>
  <div class="row"><label>AccZCal:</label><input id="AccZCalibration" style="width:120px"><button onclick="setVar('AccZCalibration')">Set</button></div>
</div>

<div class="box">
  <h3>PID Parameters</h3>
  <div class="row"><b>Angle Roll (consts, view only)</b></div>
  <div class="row"><label>PAngleRoll:</label><span id="PAngleRoll"></span></div>
  <div class="row"><label>IAngleRoll:</label><span id="IAngleRoll"></span></div>
  <div class="row"><label>DAngleRoll:</label><span id="DAngleRoll"></span></div>

  <div class="row"><b>Angle Pitch</b></div>
  <div class="row"><label>PAnglePitch:</label><input id="PAnglePitch" style="width:120px"><button onclick="setVar('PAnglePitch')">Set</button></div>
  <div class="row"><label>IAnglePitch:</label><input id="IAnglePitch" style="width:120px"><button onclick="setVar('IAnglePitch')">Set</button></div>
  <div class="row"><label>DAnglePitch:</label><input id="DAnglePitch" style="width:120px"><button onclick="setVar('DAnglePitch')">Set</button></div>

  <div class="row"><b>Rate Roll (consts partly editable)</b></div>
  <div class="row"><label>PRateRoll:</label><input id="PRateRoll" style="width:120px"><button onclick="setVar('PRateRoll')">Set</button></div>
  <div class="row"><label>IRateRoll:</label><input id="IRateRoll" style="width:120px"><button onclick="setVar('IRateRoll')">Set</button></div>
  <div class="row"><label>DRateRoll:</label><input id="DRateRoll" style="width:120px"><button onclick="setVar('DRateRoll')">Set</button></div>

  <div class="row"><b>Rate Pitch</b></div>
  <div class="row"><label>PRatePitch:</label><input id="PRatePitch" style="width:120px"><button onclick="setVar('PRatePitch')">Set</button></div>
  <div class="row"><label>IRatePitch:</label><input id="IRatePitch" style="width:120px"><button onclick="setVar('IRatePitch')">Set</button></div>
  <div class="row"><label>DRatePitch:</label><input id="DRatePitch" style="width:120px"><button onclick="setVar('DRatePitch')">Set</button></div>

  <div class="row"><b>Rate Yaw</b></div>
  <div class="row"><label>PRateYaw:</label><input id="PRateYaw" style="width:120px"><button onclick="setVar('PRateYaw')">Set</button></div>
  <div class="row"><label>IRateYaw:</label><input id="IRateYaw" style="width:120px"><button onclick="setVar('IRateYaw')">Set</button></div>
  <div class="row"><label>DRateYaw:</label><input id="DRateYaw" style="width:120px"><button onclick="setVar('DRateYaw')">Set</button></div>
</div>

<script>
let ws;
let lastSend = 0;
function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => console.log('ws open');
  ws.onclose = () => { console.log('ws close - reconnect in 1s'); setTimeout(connect,1000); };
  ws.onerror = (e) => console.log('ws err', e);
  ws.onmessage = (evt) => {
    try {
      let d = JSON.parse(evt.data);
      if (d.Receiver) document.getElementById('rcv').innerText = '['+d.Receiver.join(',')+']';
      if (d.MotorInput) document.getElementById('mot').innerText = '['+d.MotorInput.join(',')+']';
      if (d.RateRoll !== undefined) {
        document.getElementById('rRate').innerText = d.RateRoll.toFixed(2);
        document.getElementById('pRate').innerText = d.RatePitch.toFixed(2);
        document.getElementById('yRate').innerText = d.RateYaw.toFixed(2);
        document.getElementById('ax').innerText = d.AccX.toFixed(2);
        document.getElementById('ay').innerText = d.AccY.toFixed(2);
        document.getElementById('az').innerText = d.AccZ.toFixed(2);
        document.getElementById('angR').innerText = d.AngleRoll.toFixed(2);
        document.getElementById('angP').innerText = d.AnglePitch.toFixed(2);
        document.getElementById('pidR').innerText = d.PIDOutputRoll.toFixed(2);
        document.getElementById('pidP').innerText = d.PIDOutputPitch.toFixed(2);
        document.getElementById('pidY').innerText = d.PIDOutputYaw.toFixed(2);
      }
            if (d.RateCalibrationRoll !== undefined) {
    ['RateCalibrationRoll','RateCalibrationPitch','RateCalibrationYaw',
    'AccXCalibration','AccYCalibration','AccZCalibration',
    'PAnglePitch','IAnglePitch','DAnglePitch',
    'PRateRoll','IRateRoll','DRateRoll',
    'PRatePitch','IRatePitch','DRatePitch',
    'PRateYaw','IRateYaw','DRateYaw'
    ].forEach(id=>{
        if(document.getElementById(id))
            document.getElementById(id).value = d[id].toFixed(3);
    });
    document.getElementById('PAngleRoll').innerText = d.PAngleRoll.toFixed(3);
    document.getElementById('IAngleRoll').innerText = d.IAngleRoll.toFixed(3);
    document.getElementById('DAngleRoll').innerText = d.DAngleRoll.toFixed(3);
    }
    } catch(e) { console.log('parse error', e); }
  };
}

function sendCmd(s) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const now = Date.now();
  if (now - lastSend < 60) return; // basic debounce
  ws.send(s);
  lastSend = now;
}

function onRC(ch, val) {
  document.getElementById('rc'+ch+'_val').innerText = val;
  sendCmd('SET RCV ' + ch + ' ' + val);
}

function setVar(name) {
  const val = document.getElementById(name).value || '0';
  sendCmd('SET VAR ' + name + ' ' + val);
}

window.onload = connect;
</script>
</body>
</html>
)rawliteral";


// ---------- HTTP handlers ----------

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* WebSocket handler: receives frames (text). parse simple commands:
   "SET RCV <idx> <val>"
   "SET VAR <name> <val>"
*/
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    if (ws_pkt.len > 0 && ws_pkt.len < 512) {
        char *buf = malloc(ws_pkt.len + 1);
        if (!buf) return ESP_ERR_NO_MEM;
        ws_pkt.payload = (uint8_t*)buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len + 1);
        if (ret == ESP_OK) {
            buf[ws_pkt.len] = 0;
            // parse command
            // SET RCV <idx> <val>
            if (strncmp(buf, "SET RCV", 7) == 0) {
                int idx = -1, val = 0;
                if (sscanf(buf+7, "%d %d", &idx, &val) == 2) {
                    if (idx >= 0 && idx < 6) {
                        if (vars_mutex) xSemaphoreTake(vars_mutex, portMAX_DELAY);
                        ReceiverValue[idx] = val;
                        if (vars_mutex) xSemaphoreGive(vars_mutex);
                    }
                }
            } else if (strncmp(buf, "SET VAR", 7) == 0) {
    char name[64] = {0};
    double v = 0;
    if (sscanf(buf+7, "%63s %lf", name, &v) >= 1) {
        if (vars_mutex) xSemaphoreTake(vars_mutex, portMAX_DELAY);

        #define SETVAR(x) else if (strcmp(name, #x) == 0) x = (float)v;

        if (0) {}
        SETVAR(RateCalibrationRoll)
        SETVAR(RateCalibrationPitch)
        SETVAR(RateCalibrationYaw)
        SETVAR(AccXCalibration)
        SETVAR(AccYCalibration)
        SETVAR(AccZCalibration)
        SETVAR(PAnglePitch)
        SETVAR(IAnglePitch)
        SETVAR(DAnglePitch)
        SETVAR(PRateRoll)
        SETVAR(IRateRoll)
        SETVAR(DRateRoll)
        SETVAR(PRatePitch)
        SETVAR(IRatePitch)
        SETVAR(DRatePitch)
        SETVAR(PRateYaw)
        SETVAR(IRateYaw)
        SETVAR(DRateYaw)

        if (vars_mutex) xSemaphoreGive(vars_mutex);
    }       
}
        }
        free(buf);
    }
    ws_client_sock = httpd_req_to_sockfd(req);
    return ESP_OK;
}


// ---------- Broadcast task ----------

static void ws_broadcast_task(void *arg)
{
    char msg[512];
    httpd_ws_frame_t out_frame;
    out_frame.type = HTTPD_WS_TYPE_TEXT;

    while (1) {
        int rcv[6];
        int mot[4];
        float rRoll, rPitch, rYaw;
        float aX, aY, aZ;
        float aRoll, aPitch;
        float pR, pP, pY;

        if (vars_mutex) xSemaphoreTake(vars_mutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) rcv[i] = ReceiverValue[i];
        rRoll = RateRoll; rPitch = RatePitch; rYaw = RateYaw;
        aX = AccX; aY = AccY; aZ = AccZ;
        aRoll = AngleRoll; aPitch = AnglePitch;
        pR = PIDOutputRoll; pP = PIDOutputPitch; pY = PIDOutputYaw;
        mot[0] = MotorInput1; mot[1] = MotorInput2; mot[2] = MotorInput3; mot[3] = MotorInput4;
        if (vars_mutex) xSemaphoreGive(vars_mutex);

        int len = snprintf(msg, sizeof(msg),
            "{\"Receiver\":[%d,%d,%d,%d,%d,%d],"
            "\"MotorInput\":[%d,%d,%d,%d],"
            "\"RateRoll\":%.2f,\"RatePitch\":%.2f,\"RateYaw\":%.2f,"
            "\"AccX\":%.3f,\"AccY\":%.3f,\"AccZ\":%.3f,"
            "\"AngleRoll\":%.2f,\"AnglePitch\":%.2f,"
            "\"PIDOutputRoll\":%.2f,\"PIDOutputPitch\":%.2f,\"PIDOutputYaw\":%.2f}",
            rcv[0], rcv[1], rcv[2], rcv[3], rcv[4], rcv[5],
            mot[0], mot[1], mot[2], mot[3],
            rRoll, rPitch, rYaw, aX, aY, aZ, aRoll, aPitch, pR, pP, pY);

        out_frame.payload = (uint8_t*)msg;
        out_frame.len = len;

        if (ws_client_sock >= 0 && server != NULL) {
            esp_err_t r = httpd_ws_send_frame_async(server, ws_client_sock, &out_frame);
            if (r != ESP_OK) {
                ws_client_sock = -1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
}


// ---------- Server start/stop ----------

void start_webserver(void)
{
    if (vars_mutex == NULL) {
        vars_mutex = xSemaphoreCreateMutex();
        if (vars_mutex == NULL) {
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) != ESP_OK) {
        return;
    }

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws_uri);

    xTaskCreate(ws_broadcast_task, "ws_bcast", 4096, NULL, 5, NULL);
}

void stop_webserver(void)
{
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    ws_client_sock = -1;
}

void start_webserver_ap_mode(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_FC",
            .ssid_len = 0,
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();


    start_webserver();
}