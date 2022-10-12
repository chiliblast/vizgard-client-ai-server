from flask import Flask, request, jsonify, send_from_directory, abort
from flask_cors import CORS, cross_origin
import websocket
import subprocess
import re
import threading
import time
import os

from datetime import datetime
from glob import glob
import docker
import requests
import argparse
import shutil
import ssl
import pathlib

parser = argparse.ArgumentParser(description='AI agent')
parser.add_argument('--port', "-p", type=int, help='REST API port')
parser.add_argument('--home', "-d", type=str, help='Home directory')
parser.add_argument('--ssl', "-s", help='enable ssl', action="store_true")
args = parser.parse_args()


global exit_programe_flag
exit_programe_flag = False
shutdown_fortifai_flag = False
docker_api = docker.APIClient(base_url="unix://var/run/docker.sock")

rx_ws_port = re.compile(
    '^Application.ControlServerPort[\s]+=[\s]+(\\d+)', re.MULTILINE)
rx_ramdisk = re.compile(
    '^Application.SecureMediaOff[\s]+=[\s]+(\\d+)', re.MULTILINE)


class update_cpu_utilization():
    def __init__(self) -> None:
        self.cpu_sum_now = {}
        self.cpu_sum_last = {}
        self.cpu_idle_now = {}
        self.cpu_idle_last = {}
        self.cpu_used = {}

    def loop(self):
        while(not exit_programe_flag):
            with subprocess.Popen(["/bin/cat", "/proc/stat"], stdout=subprocess.PIPE) as proc:
                stddata = proc.communicate()
            usage = stddata[0].decode("utf-8")
            usage = re.findall("(cpu\d+)\s+([\d ]+)", usage, re.M)
            self.cpu_used = {}

            for u in usage:
                u0 = u[0]
                ulist = [int(i) for i in u[1].split(" ")]
                self.cpu_sum_now.update({u0: sum(ulist)})
                self.cpu_idle_now.update({u0: ulist[3]})

                if u0 not in self.cpu_sum_last.keys():
                    self.cpu_sum_last.update({u0: 0})
                if u0 not in self.cpu_idle_last.keys():
                    self.cpu_idle_last.update({u0: 0})

                cpu_delta = self.cpu_sum_now[u0] - self.cpu_sum_last[u0]
                cpu_idle = self.cpu_idle_now[u0] - self.cpu_idle_last[u0]
                self.cpu_used.update(
                    {u0: round(1e4 * (1-(cpu_idle/cpu_delta)))/100})
                self.cpu_sum_last.update({u0: self.cpu_sum_now[u0]})
                self.cpu_idle_last.update({u0: self.cpu_idle_now[u0]})
            time.sleep(1)

    def getter(self):
        return self.cpu_used


class ai_server_status():
    def __init__(self) -> None:
        self.status = 0

    def loop(self):
        while(not exit_programe_flag):
            with subprocess.Popen(["/usr/bin/docker", "ps"], stdout=subprocess.PIPE) as proc:
                stddata = proc.communicate()
            status = stddata[0].decode("utf-8").strip()
            if len(re.findall("(ai_server$)", status, re.M)) == 1:
                self.status = 2
            elif len(re.findall("(ai_server_config$)", status, re.M)) == 1:
                self.status = 1
            else:
                self.status = 0
            time.sleep(1)

    def getter(self):
        if self.status == 0:
            return "stopped"
        elif self.status == 1:
            return "loading"
        elif self.status == 2:
            return "running"


class pull_image():
    def __init__(self) -> None:
        self.reset_status()

    def reset_status(self):
        self.current = 0
        self.total = 0
        self.percentage = 0
        self.done = True

    def pull(self, image, new_tag, reboot=False, copy="", command=""):
        try:
            if self.done:
                self.current = 0
                self.total = 0
                layer = {}
                percentage_per_step = 0
                for line in docker_api.pull(image, stream=True, decode=True):
                    self.done = False
                    if "progressDetail" in line.keys():
                        layer.update({line["id"]: {"status": line["status"]}})
                        if ("current" in line["progressDetail"].keys()) and ("total" in line["progressDetail"].keys()):
                            layer[line["id"]].update(
                                {"progress": line["progressDetail"]["current"] / line["progressDetail"]["total"]})
                        else:
                            layer[line["id"]].update({"progress": 0.})
                        self.total = (len(layer.keys()))
                        percentage_per_step = 100. / self.total
                    _current = 0
                    _percentage = 0
                    for k in layer.keys():
                        if (layer[k]["status"] == "Already exists") or (layer[k]["status"] == "Pull complete"):
                            _current += 1
                            _percentage += percentage_per_step
                        else:
                            _percentage += layer[k]["progress"]

                    self.current = _current
                    self.percentage = round(_percentage * 100) / 100
                    self.percentage = 100. if self.percentage > 100 else self.percentage

                self.current = self.total
                self.done = True
                docker_api.tag(image, new_tag)
        except Exception as e:
            self.done = True
        finally:
            self.done = True
        if command != "":
            print(command)
            os.system(command)
        if reboot:
            if args.ssl:
                requests.post(f"https://127.0.0.1:{args.port}/start/default{copy}")
            else:
                requests.post(f"http://127.0.0.1:{args.port}/start/default{copy}")
        else:
            time.sleep(5)
        self.reset_status()

    def getter(self):
        return {"current": self.current, "total": self.total, "done": self.done, "percentage": self.percentage}

    def get_latest_version(self, repo, tag=None):
        if tag is None:
            tag = "latest"
        try:
            r = requests.get(f'https://auth.docker.io/token', params={
                             'service': "registry.docker.io", "scope": f"repository:{repo}:pull"})
            token = r.json()["token"]
            header = {"Accept": "application/vnd.docker.distribution.manifest.v2+json,application/vnd.docker.distribution.manifest.list.v2+json",
                      "Authorization": f"Bearer {token}"}
            r = requests.get(
                f'https://registry-1.docker.io/v2/{repo}/manifests/{tag}', headers=header)
            if "errors" in r.json().keys():
                return None
            digest = r.json()["config"]["digest"]
            return digest
        except Exception as e:
            print(e)
            return None

    def get_local_version(self, repo, tag=None):
        if tag is None:
            tag = "latest"
        try:
            local_images = docker_api.images()
            local_images_id = []
            for li in local_images:
                if li["RepoDigests"] is not None:
                    for digest in li["RepoDigests"]:
                        if f"{repo}" in digest:
                            local_images_id.append(li["Id"])
                            break
        except Exception as e:
            print(e)
            return None
        return local_images_id

    def check_for_update(self, repo, tag=None):
        if tag is None:
            tag = "latest"
        latest_version = self.get_latest_version(repo, tag)
        local_version = self.get_local_version(repo)
        print(local_version)
        return (latest_version in local_version), latest_version


def ws_connect(ws_port):
    try:
        if args.ssl:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            ssl_context.load_verify_locations(f"{args.home}/Software/cc/server")
            # ssl_context.check_hostname = False
            # ssl_context.verify_mode = ssl.CERT_NONE
            ws = websocket.WebSocket(sslopt={"context": ssl_context})
            ws.connect(f"wss://127.0.0.1:{ws_port}")
            # ws = websocket.create_connection(f"wss://127.0.0.1:{ws_port}", ssl=ssl_context)
        else:
            ws = websocket.create_connection(f"ws://127.0.0.1:{ws_port}")
    except OSError as e:
        ws = None
    except websocket._exceptions.WebSocketConnectionClosedException as e:
        ws = None
    except websocket._exceptions.WebSocketBadStatusException as e:
        ws = None
    except Exception as e:
        print(e)
        ws = None
    return ws


def get_ws_connection(ws_port=None):
    ws = None
    if ws_port is not None:
        ws = ws_connect(ws_port)

    if ws is None:
        with open(os.path.join(f"{args.home}/Software/cc", os.path.basename(os.readlink(f"{args.home}/Software/cc/ai_server.cfg"))), mode="r", encoding="utf-8") as f:
            data = f.read()

        ws_port = [m.groups() for m in rx_ws_port.finditer(data)]
        if len(ws_port) > 0:
            ws_port = int(ws_port[0][0])
            ws = ws_connect(ws_port)
        else:
            ws = None

    return ws


def control_system():
    while(not exit_programe_flag):
        time.sleep(10)
        try:
            ws = get_ws_connection()
            if ws is None:
                continue
            if args.ssl:
                print("connection to wss://127.0.0.1:\{ws_port\} was established")
            else:
                print("connection to ws://127.0.0.1:\{ws_port\} was established")
            while 1:
                data = ws.recv()
                # print(data, datetime.now())
                if data == "reboot-jetson":
                    print("reboot-jetson", datetime.now())
                    os.system("/sbin/reboot 0")
                if data == "shutdown-jetson":
                    print("shutdown-jetson", datetime.now())
                    os.system("/sbin/shutdown -h now")
        except OSError as e:
            print(e, datetime.now())
        except websocket._exceptions.WebSocketConnectionClosedException as e:
            print(e, datetime.now())


def move_data_from_ramdisk():
    previous_ramdisk_off = None
    while(not exit_programe_flag):
        time.sleep(1)
        with open(os.path.join(f"{args.home}/Software/cc", os.path.basename(os.readlink(f"{args.home}/Software/cc/ai_server.cfg"))), mode="r", encoding="utf-8") as f:
            data = f.read()
        ramdisk_off = [m.groups() for m in rx_ramdisk.finditer(data)]
        if len(ramdisk_off) > 0:
            ramdisk_off = int(ramdisk_off[0][0])
            if ramdisk_off and (not previous_ramdisk_off):
                for fn in glob(f"{args.home}/Software/cc/Ramdisk/**", recursive=True):
                    if os.path.isfile(fn):
                        shutil.move(fn, fn.replace(
                            f"{args.home}/Software/cc/Ramdisk/", f"{args.home}/Software/cc/"))
                    elif os.path.isdir(fn):
                        fn = fn.replace(
                            f"{args.home}/Software/cc/Ramdisk/", f"{args.home}/Software/cc/")
                        os.system(f"mkdir -p {fn}")
            previous_ramdisk_off = ramdisk_off
        else:
            continue


cpu = update_cpu_utilization()
vstatus = ai_server_status()
ipull = pull_image()
t1 = threading.Thread(target=cpu.loop)
t1.start()
t2 = threading.Thread(target=vstatus.loop)
t2.start()
t3 = threading.Thread(target=control_system)
t3.start()
t4 = threading.Thread(target=move_data_from_ramdisk)
t4.start()

app = Flask(__name__)
CORS(app, support_credentials=True)


@app.route("/", methods=["GET"])
@cross_origin(supports_credentials=True)
def root():
    return {"message": "ai_server"}


@app.route("/jetson_state", methods=["GET"])
@cross_origin(supports_credentials=True)
def jetson_state():
    jetson = {}
    with subprocess.Popen(["/usr/bin/docker", "ps"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    status = stddata[0].decode("utf-8").strip()
    if len(re.findall("(ai_server$)", status, re.M)) == 1:
        jetson.update({"status": "running"})
    elif len(re.findall("(ai_server_config$)", status, re.M)) == 1:
        jetson.update({"status": "loading"})
    else:
        jetson.update({"status": "stopped"})

    with subprocess.Popen(["/bin/df", "-h"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    storage = stddata[0].decode("utf-8").strip()
    usage = re.findall("\d+(?=% /$)", storage, re.M)
    jetson.update({"storage": {"value": int(usage[0]), "unit": "%"}})

    ramdisk = stddata[0].decode("utf-8").strip()
    usage = re.findall(
        f"\d+(?=% {args.home}/Software/cc/Ramdisk$)", ramdisk, re.M)
    jetson.update({"ramdisk": {"value": int(usage[0]), "unit": "%"}})

    with subprocess.Popen(["/bin/cat", "/sys/devices/virtual/thermal/thermal_zone0/temp"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    temp = int(stddata[0].decode("utf-8").strip())
    temp = round((temp / 1000)*100)/100
    jetson.update({"cpu_temp": {"value": temp, "unit": "deg C"}})

    with subprocess.Popen(["/bin/cat", "/sys/devices/virtual/thermal/thermal_zone1/temp"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    temp = int(stddata[0].decode("utf-8").strip())
    temp = round((temp / 1000)*100)/100
    jetson.update({"gpu_temp": {"value": temp, "unit": "deg C"}})

    with subprocess.Popen(["/bin/cat", "/sys/devices/gpu.0/load"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    temp = int(stddata[0].decode("utf-8").strip())
    temp = (temp / 10.0)
    jetson.update({"gpu_usage": {"value": temp, "unit": "%"}})

    rcpu = cpu.getter()
    rcpu.update({"unit": "%"})
    jetson.update({"cpu_usage": rcpu})

    with subprocess.Popen(["/bin/cat", "/sys/devices/generic_pwm_tachometer/hwmon/hwmon1/rpm"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    rpm = int(stddata[0].decode("utf-8").strip())
    fan_speed = round(rpm*10000 / 3400)/100
    jetson.update({"fan_speed": {"value": fan_speed, "unit": "%"}})

    with open("/proc/device-tree/serial-number", mode="rb") as f:
        device_serial_number = f.read()
        jetson.update(
            {"serial_number": device_serial_number.decode("UTF-8")[:-1]})

    list_cfg = []
    for f in glob(f"{args.home}/Software/cc/*.cfg"):
        if os.path.basename(f) == "ai_server.cfg":
            if os.path.islink(f):
                continue
            else:
                os.remove(f)
        else:
            list_cfg.append(os.path.basename(f))
    jetson.update({"list-config": list_cfg})
    jetson.update(
        {"license-valid": os.path.exists(f"{args.home}/Software/cc/ai_server.lic")})
    jetson.update({"fortifai-version": "V1.1.0RC"})
    jetson.update({"pull_progress": ipull.getter()})
    jetson.update(
        {"actual-config-file": os.path.basename(os.readlink(f"{args.home}/Software/cc/ai_server.cfg"))})
    with subprocess.Popen(["/usr/bin/free", "-m"], stdout=subprocess.PIPE) as proc:
        stddata = proc.communicate()
    stddata = stddata[0].decode("utf-8")
    ram, ram_usage = re.findall("Mem:\s+(\d+)\s+(\d+)", stddata, re.M)[0]
    swap, swap_usage = re.findall("Swap:\s+(\d+)\s+(\d+)", stddata, re.M)[0]
    jetson.update(
        {"memory": {"value": round((int(ram_usage) / 102.4)) / 10, "unit": "GB"}})
    jetson.update(
        {"swap": {"value": round((int(swap_usage) / 102.4)) / 10, "unit": "GB"}})

    with open(os.path.join(f"{args.home}/Software/cc", os.path.basename(os.readlink(f"{args.home}/Software/cc/ai_server.cfg"))), mode="r", encoding="utf-8") as f:
        data = f.read()
    ws_port = [m.groups() for m in rx_ws_port.finditer(data)]
    if len(ws_port) > 0:
        jetson.update({"ws_port": int(ws_port[0][0])})
    else:
        jetson.update({"ws_port": None})
    return jetson


def check_valid_app_config():
    if args.ssl:
        r = requests.get(f"https://127.0.0.1:{args.port}/get-local-version")
    else:
        r = requests.get(f"http://127.0.0.1:{args.port}/get-local-version")
    r = r.json()
    valid_app = False
    valid_cfg = False
    for i in r["ai_server-app"]:
        valid_app |= i["default"]
    for i in r["ai_server-config"]:
        valid_cfg |= i["default"]
    return valid_app and valid_cfg


@app.route("/start/<path:path>", methods=["POST"])
@cross_origin(supports_credentials=True)
def start(path):
    if not ((path == "default") or (path == "default-c") or (path == "default--copy")):
        if not os.path.exists(os.path.join(f"{args.home}/Software/cc", path)):
            abort(404)
    if not ((path == "default") or (path == "default-c") or (path == "default--copy")):
        os.system(
            f"cd {args.home}/Software/cc/ && rm -rf ai_server.cfg && ln -sf {path} ai_server.cfg")

    s = vstatus.getter()
    print("\tSSS: ",s)
    if s == "running":
        return {"response": "Config Updated. Please Reboot FortifAI"}
    elif s == "loading":
        return {"response": "Config Updated. Please Reboot FortifAI"}
    else:
        if not check_valid_app_config():
            return {"response": "Have not select version of ai_server_config or ai_server_app"}
        
        print(f"bash {args.home}/Software/run_docker.sh -d {path.replace('default', '')}")
        with subprocess.Popen([f"/bin/bash", f"{args.home}/Software/run_docker.sh", f"-d {path.replace('default', '')}"], stdout=subprocess.PIPE) as proc:
            stddata = proc.communicate()
        container_id = stddata[0].decode("utf-8").strip()
        print("container_id:", container_id)
        return {"response": "Starting"}


@app.route("/fortifai-reboot/<path:path>", methods=["POST"])
@cross_origin(supports_credentials=True)
def reboot_fortifai(path):
    global shutdown_fortifai_flag
    if not shutdown_fortifai_flag:
        shutdown_fortifai_flag = True
        if not path == "default":
            if not os.path.exists(os.path.join(f"{args.home}/Software/cc", path)):
                abort(404)

        s = vstatus.getter()
        if (s == "running") or (s == "loading"):
            for i in range(10):
                try:
                    # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                    ws = get_ws_connection(request.args.get('p'))
                    if ws is not None:
                        ws.send("shutdown")
                    break
                except OSError as e:
                    time.sleep(0.5)

        time.sleep(3)
        os.system("/usr/bin/docker stop ai_server")
        if not path == "default":
            os.system(
                f"cd {args.home}/Software/cc/ && rm -rf ai_server.cfg && ln -sf {path} ai_server.cfg")
        if not check_valid_app_config():
            shutdown_fortifai_flag = False
            return {"response": "Have not select version of ai_server_config or ai_server_app"}
        with subprocess.Popen(["bash", f"{args.home}/Software/run_docker.sh", "-d"], stdout=subprocess.PIPE) as proc:
            stddata = proc.communicate()
        container_id = stddata[0].decode("utf-8").strip()
        print("container_id:", container_id)
        shutdown_fortifai_flag = False
        return jsonify({"response": "reboot-fortifai"})
    else:
        print("ELSE: ----fortifai-reboot")
        return jsonify({"response": "reboot-fortifai"})


@app.route("/fortifai-shutdown", methods=["POST"])
@cross_origin(supports_credentials=True)
def shutdown_fortifai():
    global shutdown_fortifai_flag
    if not shutdown_fortifai_flag:
        shutdown_fortifai_flag = True
        s = vstatus.getter()
        if (s == "running") or (s == "loading"):
            for i in range(10):
                try:
                    # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                    ws = get_ws_connection(request.args.get('p'))
                    if ws is not None:
                        ws.send("shutdown")
                    break
                except OSError as e:
                    time.sleep(0.5)

        time.sleep(3)
        os.system("/usr/bin/docker stop ai_server")
        shutdown_fortifai_flag = False
        return jsonify({"response": "shutdown-fortifai"})
    else:
        return jsonify({"response": "shutdown-fortifai"})


@app.route("/start2/<path:path>", methods=["POST"])
@cross_origin(supports_credentials=True)
def start2(path):
    if not ((path == "default") or (path == "default-c") or (path == "default--copy")):
        if not os.path.exists(os.path.join(f"{args.home}/Software/cc", path)):
            abort(404)
    if not ((path == "default") or (path == "default-c") or (path == "default--copy")):
        os.system(
            f"cd {args.home}/Software/cc/ && rm -rf ai_server.cfg && ln -sf {path} ai_server.cfg")

    s = vstatus.getter()
    if s == "running":
        return {"response": "Config Updated. Please Reboot FortifAI"}
    elif s == "loading":
        return {"response": "Config Updated. Please Reboot FortifAI"}
    else:
        if not check_valid_app_config():
            return {"response": "Have not select version of ai_server_config or ai_server_app"}
        with subprocess.Popen(["bash", f"{args.home}/Software/run_docker.sh", "-d", "-w", path.replace("default", "")], stdout=subprocess.PIPE) as proc:
            stddata = proc.communicate()
        container_id = stddata[0].decode("utf-8").strip()
        print("container_id:", container_id)
        return {"response": "Starting"}


@app.route("/fortifai-reboot2/<path:path>", methods=["POST"])
@cross_origin(supports_credentials=True)
def reboot_fortifai2(path):
    global shutdown_fortifai_flag
    if not shutdown_fortifai_flag:
        shutdown_fortifai_flag = True
        if not path == "default":
            if not os.path.exists(os.path.join(f"{args.home}/Software/cc", path)):
                abort(404)

        s = vstatus.getter()
        if (s == "running") or (s == "loading"):
            for i in range(10):
                try:
                    # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                    ws = get_ws_connection(request.args.get('p'))
                    if ws is not None:
                        ws.send("shutdown")
                    break
                except OSError as e:
                    time.sleep(0.5)

        time.sleep(3)
        os.system("/usr/bin/docker stop ai_server_webrtc")
        if not path == "default":
            os.system(
                f"cd {args.home}/Software/cc/ && rm -rf ai_server.cfg && ln -sf {path} ai_server.cfg")
        if not check_valid_app_config():
            shutdown_fortifai_flag = False
            return {"response": "Have not select version of ai_server_config or ai_server_app"}
        with subprocess.Popen(["bash", f"{args.home}/Software/run_docker.sh", "-d", "-s"], stdout=subprocess.PIPE) as proc:
            stddata = proc.communicate()
        container_id = stddata[0].decode("utf-8").strip()
        print("container_id:", container_id)
        shutdown_fortifai_flag = False
        return jsonify({"response": "reboot-fortifai"})
    else:
        print("ELSE: ----fortifai-reboot")
        return jsonify({"response": "reboot-fortifai"})


@app.route("/fortifai-shutdown2", methods=["POST"])
@cross_origin(supports_credentials=True)
def shutdown_fortifai2():
    global shutdown_fortifai_flag
    if not shutdown_fortifai_flag:
        shutdown_fortifai_flag = True
        s = vstatus.getter()
        if (s == "running") or (s == "loading"):
            for i in range(10):
                try:
                    # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                    ws = get_ws_connection(request.args.get('p'))
                    if ws is not None:
                        ws.send("shutdown")
                    break
                except OSError as e:
                    time.sleep(0.5)

        time.sleep(3)
        os.system("/usr/bin/docker stop ai_server_webrtc")
        shutdown_fortifai_flag = False
        return jsonify({"response": "shutdown-fortifai"})
    else:
        return jsonify({"response": "shutdown-fortifai"})


@app.route("/reboot", methods=["POST"])
@cross_origin(supports_credentials=True)
def reboot():
    s = vstatus.getter()
    if (s == "running") or (s == "loading"):
        for i in range(10):
            try:
                # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                ws = get_ws_connection(request.args.get('p'))
                if ws is not None:
                    ws.send("shutdown")
                break
            except OSError as e:
                time.sleep(0.5)

    time.sleep(3)
    if ((vstatus.getter() == "running") or (vstatus.getter() == "loading")):
        os.system("/usr/bin/docker stop ai_server")
    os.system("/sbin/reboot 0")
    return jsonify({"response": "reboot-jetson"})


@app.route("/shutdown", methods=["POST"])
@cross_origin(supports_credentials=True)
def shutdown():
    s = vstatus.getter()
    if (s == "running") or (s == "loading"):
        for i in range(10):
            try:
                # ws = websocket.create_connection(f"ws://127.0.0.1:{request.args.get('p')}")
                ws = get_ws_connection(request.args.get('p'))
                if ws is not None:
                    ws.send("shutdown")
                break
            except OSError as e:
                time.sleep(0.5)

    time.sleep(3)
    if ((vstatus.getter() == "running") or (vstatus.getter() == "loading")):
        os.system("/usr/bin/docker stop ai_server")
    os.system("/sbin/shutdown -h now")
    return jsonify({"response": "shutdown-jetson"})


@app.route("/list-config", methods=["GET"])
@cross_origin(supports_credentials=True)
def list_config():
    list_cfg = []
    for f in glob(f"{args.home}/Software/cc/*.cfg"):
        if os.path.basename(f) == "ai_server.cfg":
            continue
        else:
            list_cfg.append(os.path.basename(f))
    return {"list-config": list_cfg}


@app.route("/get-file/<path:path>", methods=["GET"])
@cross_origin(supports_credentials=True)
def get_file(path):
    try:
        return send_from_directory(f"{args.home}/Software/cc", path, as_attachment=True)
    except FileNotFoundError:
        abort(404)


@app.route("/update-file/<path:path>", methods=["POST"])
@cross_origin(supports_credentials=True)
def update_file(path):
    f = request.files["file"]
    f.save(os.path.join(f"{args.home}/Software/cc", path))
    return {"uploaded": path}


@app.route("/delete-file/<path:path>", methods=["DELETE"])
@cross_origin(supports_credentials=True)
def delete_file(path):
    os.system(f"rm -rf {args.home}/Software/cc/{path}")
    return {"removed": path}


@app.route("/update-ai_server-app", methods=["POST"])
@cross_origin(supports_credentials=True)
def update_ai_server_app():
    tag = str(request.args.get("tag"))
    if ipull.getter()["done"]:
        s = vstatus.getter()
        if "p" in dict(request.args).keys():
            if (s == "running") or (s == "loading"):
                if args.ssl:
                    requests.post(f"https://127.0.0.1:{args.port}/fortifai-shutdown?p={str(request.args.get('p'))}")
                else:
                    requests.post(f"http://127.0.0.1:{args.port}/fortifai-shutdown?p={str(request.args.get('p'))}")
            for i in range(40):
                if (vstatus.getter() == "running"):
                    time.sleep(0.5)
            if (vstatus.getter() == "running"):
                return {"status": "Can not stop Fortifai, please close it manually!"}

        if ipull.getter()["done"]:
            b = ((s == "running") or (s == "loading"))
            t = threading.Thread(target=ipull.pull, args=(
                f"alexvizgard/ai_server_app:{tag}", "ai_server_app:latest", (True if b else False), "", ""))
            t.start()
            return {"status": "Update is in progress"}
        else:
            return {"status": "Other process is not finished!"}
    # os.system(f"/usr/bin/docker pull alexvizgard/ai_server_app:{tag} && /usr/bin/docker tag alexvizgard/ai_server_app:{tag} ai_server_app:latest")


@app.route("/update-ai_server-config", methods=["POST"])
@cross_origin(supports_credentials=True)
def update_ai_server_config():
    tag = str(request.args.get("tag"))
    s = vstatus.getter()
    if ipull.getter()["done"]:
        if "p" in dict(request.args).keys():
            if (s == "running") or (s == "loading"):
                if args.ssl:
                    requests.post(f"https://127.0.0.1:{args.port}/fortifai-shutdown?p={str(request.args.get('p'))}")
                else:
                    requests.post(f"http://127.0.0.1:{args.port}/fortifai-shutdown?p={str(request.args.get('p'))}")
            for i in range(40):
                if (vstatus.getter() == "running"):
                    time.sleep(0.5)
            if (vstatus.getter() == "running"):
                return {"status": "Can not stop Fortifai, please close it manually!"}
        b = ((s == "running") or (s == "loading")) and (
            "p" in dict(request.args).keys())
        t = threading.Thread(target=ipull.pull, args=(f"alexvizgard/ai_server_config:{tag}", "ai_server_config:v", (
            True if b else False), "--copy", "docker run -it -d --rm --name ai_server_config -v ai_server_volume:/root/ai_server/temp ai_server_config:v"))
        t.start()
        return {"status": "updated successfully"}
    else:
        return {"status": "Other process is not finished!"}
    # os.system(f"/usr/bin/docker pull alexvizgard/ai_server_config:{tag} && /usr/bin/docker tag alexvizgard/ai_server_config:{tag} ai_server_config:vv")


@app.route("/check-for-update-ai_server-app", methods=["POST"])
@cross_origin(supports_credentials=True)
def check_for_update_ai_server_app():
    latest, Id = ipull.check_for_update(
        "alexvizgard/ai_server_app", str(request.args.get("tag")))
    if Id is None:
        return {"updateavailable": False,
                "shortID": None,
                "warning": "check limit reached"}
    else:
        return {"updateavailable": not latest,
                "shortID": Id.replace("sha256:", "")[:12]}


@app.route("/check-for-update-ai_server-config", methods=["POST"])
@cross_origin(supports_credentials=True)
def check_for_update_ai_server_config():
    latest, Id = ipull.check_for_update(
        "alexvizgard/ai_server_config", str(request.args.get("tag")))
    if Id is None:
        return {"updateavailable": False,
                "shortID": None,
                "warning": "check limit reached"}
    else:
        return {"updateavailable": not latest,
                "shortID": Id.replace("sha256:", "")[:12]}


def timedelta_to_string(delta_t):
    if delta_t.days == 0:
        if delta_t.seconds == 1:
            return f"1 second ago"
        elif delta_t.seconds < 120:
            return f"{delta_t.seconds} seconds ago"
        elif delta_t.seconds < 7200:
            return f"{delta_t.seconds//60} minutes ago"
        else:
            return f"{delta_t.seconds//3600} hours ago"
    else:  # delta_t.days > 0
        if delta_t.days == 1:
            return f"1 day ago"
        elif delta_t.days < 14:
            return f"{delta_t.days} days ago"
        elif delta_t.days < 70:
            return f"{delta_t.days//7} weeks ago"
        else:
            return f"{delta_t.days//30} months ago"


def get_repo_tag(image_info):
    r = image_info["RepoDigests"]
    t = image_info["RepoTags"]
    if r is None:
        return t[0].split(":")
    elif t is None:
        return r[0].split("@")[0], "<none>"
    elif (r[0] == "<none>@<none>") and (t[0] == "<none>:<none>"):
        return "<none>", "<none>"
    else:
        return t[0].split(":")


def check_local_latest_version(tag, RepoTags):
    if RepoTags is not None:
        for _tag in RepoTags:
            if tag == _tag:
                latest_flag = True
                break
            else:
                latest_flag = False
    else:
        latest_flag = False
    return latest_flag


@app.route("/get-local-version", methods=["GET"])
@cross_origin(supports_credentials=True)
def get_local_version():
    app_versions = []
    cfg_versions = []
    remote_version = []
    default_repo_digest = None
    list_repo_digest = []

    for image_info in docker_api.images():
        if "./vizgard" in docker_api.history(image_info["Id"])[0]["CreatedBy"]:
            app_versions.append({"Repo": get_repo_tag(image_info)[0],
                                "Tag": get_repo_tag(image_info)[1],
                                "Id": image_info["Id"],
                                "ShortId": image_info["Id"].replace("sha256:", "")[:12],
                                "Created": datetime.fromtimestamp(image_info["Created"]).strftime("%H:%M %d/%m/d%Y"),
                                "Size": {"value": image_info["Size"] // 10000000 / 100, "unit": "GB"},
                                "RepoDigests": "" if image_info["RepoDigests"] is None else image_info["RepoDigests"][0],
                                "default": check_local_latest_version("ai_server_app:latest", image_info["RepoTags"])})
            list_repo_digest.append(app_versions[-1]["RepoDigests"].replace("alexvizgard/ai_server_app@", ""))
            if app_versions[-1]["default"]:
                default_repo_digest = app_versions[-1]["RepoDigests"].replace("alexvizgard/ai_server_app@", "")
            
        elif "mCopy.py" in docker_api.history(image_info["Id"])[0]["CreatedBy"]:
            cfg_versions.append({"Repo": get_repo_tag(image_info)[0],
                                "Tag": get_repo_tag(image_info)[1],
                                "Id": image_info["Id"],
                                "ShortId": image_info["Id"].replace("sha256:", "")[:12],
                                "Created": datetime.fromtimestamp(image_info["Created"]).strftime("%H:%M %d/%m/%Y"),
                                "Size": {"value": image_info["Size"] // 10000000 / 100, "unit": "GB"},
                                "RepoDigests": image_info["RepoDigests"],
                                "default": check_local_latest_version("ai_server_config:v", image_info["RepoTags"])})

    try:
        r = requests.get("https://registry.hub.docker.com/v2/repositories/alexvizgard/ai_server_app/tags").json()
        if "errors" not in r.keys():
            for res in r["results"]:
                if res["name"].startswith("V"):
                    y, m, d, H, M = re.findall("(\d+)-(\d+)-(\d+)T(\d+):(\d+).*(?=Z)Z$", res["last_updated"])[0]
                    remote_version.append({"tag": res["name"],
                                            "digest": res["images"][0]["digest"],
                                            "last_updated": f"{H}:{M} {d}/{m}/{y}",
                                            "default": (res["images"][0]["digest"] == default_repo_digest),
                                            "downloaded": res["images"][0]["digest"] in list_repo_digest})
                    print(f"default_repo_digest: {default_repo_digest} - {res['images'][0]['digest']}")
    except Exception as e:
        remote_version = []
    
    return {"ai_server-app": app_versions, "ai_server-config": cfg_versions, "remote-ai_server-app": remote_version}


@app.route("/set-ai_server-app-version", methods=["POST"])
@cross_origin(supports_credentials=True)
def set_ai_server_app_version():
    # import ipdb; ipdb.set_trace()
    request.args.get('Id')
    if args.ssl:
        r = requests.get(f"https://127.0.0.1:{args.port}/get-local-version")
    else:
        r = requests.get(f"http://127.0.0.1:{args.port}/get-local-version")
    valid_id = False
    for img in (r.json()["ai_server-app"] + r.json()["ai_server-config"]):
        if request.args.get('Id') == img["ShortId"]:
            valid_id = True
            break
    if not valid_id:
        return {"status": "Invalid image id"}
    os.system(
        f"/usr/bin/docker tag {request.args.get('Id')} ai_server_app:latest")
    s = vstatus.getter()
    print("p" in dict(request.args).keys())
    b = ((s == "running") or (s == "loading")) and (
        "p" in dict(request.args).keys())
    if ((s == "running") or (s == "loading")) and ("p" in dict(request.args).keys()):
        if args.ssl:
            requests.post(f"https://127.0.0.1:{args.port}/fortifai-reboot/default?p={str(request.args.get('p'))}")
        else:
            requests.post(f"http://127.0.0.1:{args.port}/fortifai-reboot/default?p={str(request.args.get('p'))}")

        for i in range(40):
            if (vstatus.getter() == "running"):
                time.sleep(0.5)
        if (vstatus.getter() == "running"):
            return {"status": f"Changed ai_server-app version to {request.args.get('Id')}. Can not stop Fortifai, please close it manually and relaunch again!"}
        else:
            return {"status": f"Changed ai_server-app version to {request.args.get('Id')} and relaunched!"}
    return {"status": f"Changed ai_server-app version to {request.args.get('Id')}"}


@app.route("/set-ai_server-config-version", methods=["POST"])
@cross_origin(supports_credentials=True)
def set_ai_server_config_version():
    request.args.get('Id')
    if args.ssl:
        r = requests.get(f"https://127.0.0.1:{args.port}/get-local-version")
    else:
        r = requests.get(f"http://127.0.0.1:{args.port}/get-local-version")
    valid_id = False
    for img in (r.json()["ai_server-app"] + r.json()["ai_server-config"]):
        if request.args.get('Id') == img["ShortId"]:
            valid_id = True
            break
    if not valid_id:
        return {"status": "Invalid image id"}
    os.system(f"/usr/bin/docker tag {request.args.get('Id')} ai_server_config:v")
    s = vstatus.getter()
    b = ((s == "running") or (s == "loading")) and (
        "p" in dict(request.args).keys())
    if ((s == "running") or (s == "loading")) and ("p" in dict(request.args).keys()):
        if args.ssl:
            requests.post(f"https://127.0.0.1:{args.port}/fortifai-reboot/default?p={str(request.args.get('p'))}")
        else:
            requests.post(f"http://127.0.0.1:{args.port}/fortifai-reboot/default?p={str(request.args.get('p'))}")
        for i in range(40):
            if (vstatus.getter() == "running"):
                time.sleep(0.5)
        if (vstatus.getter() == "running"):
            return {"status": f"Changed ai_server-config version to {request.args.get('Id')}. Can not stop Fortifai, please close it manually and relaunch again!"}
        else:
            return {"status": f"Changed ai_server-config version to {request.args.get('Id')} and relaunched!"}
    return {"status": f"Changed ai_server-config version to {request.args.get('Id')}"}


@app.route("/delete-by-short-id", methods=["DELETE"])
@cross_origin(supports_credentials=True)
def delete_image_by_id():
    try:
        docker_api.remove_image(image=request.args.get('Id'), force=True)
        return {"status": f"removed {request.args.get('Id')}"}
    except docker.errors.APIError as e:
        return {"status": f"{request.args.get('Id')} is running or there is some issue occured"}


if __name__ == "__main__":
    if args.ssl:
        context = (f"{args.home}/Software/cc/server/ai_server.com.crt", f"{args.home}/Software/cc/server/ai_server.com.key")
        app.run(host="0.0.0.0", debug=True, port=args.port, ssl_context=context)
    else:
        app.run(host="0.0.0.0", port=args.port)
    exit_programe_flag = True
    t1.join()
    t2.join()
    t3.join()
    t4.join()
