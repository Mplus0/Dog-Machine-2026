#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import datetime as _dt
import getpass
import os
import shlex
import signal
import struct
import subprocess
import sys
import time


DEFAULT_PORTS = [43893, 43894, 43897, 43899, 8554]
DEFAULT_PRIVATE_CONFIG = "private_robot_access.yaml"


def shell_join(args):
    return " ".join(shlex.quote(str(arg)) for arg in args)


def read_simple_yaml(path):
    data = {}
    if not path or not os.path.exists(path):
        return data
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            value = value.strip()
            if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
                value = value[1:-1]
            data[key.strip()] = value
    return data


def apply_private_config(args):
    config = read_simple_yaml(args.private_config)
    if not config:
        return
    args.motion_host = config.get("motion_host", args.motion_host)
    args.motion_host_p2p = config.get("motion_host_p2p", args.motion_host_p2p)
    args.handheld_ip = config.get("handheld_ip", args.handheld_ip)
    args.tablet_ip = config.get("tablet_ip", args.tablet_ip)
    args.ssh_user = config.get("motion_host_ssh_user", config.get("ssh_user", args.ssh_user))
    args.ssh_password = config.get("motion_host_ssh_password", config.get("ssh_password", args.ssh_password))
    args.sudo_password = config.get("motion_host_sudo_password", config.get("sudo_password", args.sudo_password))
    args.ssh = config.get("motion_host_ssh", config.get("ssh", args.ssh))
    if args.sshpass is None:
        args.sshpass = parse_bool(config.get("sshpass", False))
    if args.sudo is None:
        args.sudo = parse_bool(config.get("sudo", False))
    if args.sudo_with_password is None:
        args.sudo_with_password = parse_bool(config.get("sudo_with_password", False))
    if parse_bool(config.get("sudo_password_same_as_ssh", False)) and not args.sudo_password:
        args.sudo_password = args.ssh_password


def parse_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off"):
        return False
    return bool(value)


def redact_command_for_print(command):
    redacted = []
    skip_next = False
    for item in command:
        if skip_next:
            redacted.append("***")
            skip_next = False
            continue
        redacted.append(item)
        if item in ("-p", "--password"):
            skip_next = True
    return redacted


def build_filter(args):
    terms = []
    filter_hosts = args.filter_host[:]
    if not filter_hosts and args.motion_host:
        filter_hosts.append(args.motion_host)
    for host in filter_hosts:
        terms.append("host %s" % host)
    for host in args.host:
        terms.append("host %s" % host)
    port_terms = [] if args.all_ports else ["port %d" % port for port in args.port]
    if port_terms:
        terms.append("(" + " or ".join(port_terms) + ")")
    if args.udp_only:
        terms.append("udp")
    if args.tcp_only:
        terms.append("tcp")
    if not terms:
        return " or ".join("port %d" % port for port in DEFAULT_PORTS)
    return " and ".join(terms)


def default_output_path(prefix, suffix=".pcap"):
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.abspath(os.path.expanduser("~/packet_captures"))
    return os.path.join(output_dir, "%s_%s%s" % (prefix, stamp, suffix))


def build_tcpdump_args(args, output):
    command = []
    if args.sudo:
        command.append("sudo")
    command.extend(
        [
            "tcpdump",
            "-i",
            args.interface,
            "-s",
            str(args.snaplen),
            "-U",
            "-nn",
            "-w",
            output,
        ]
    )
    if args.count > 0:
        command.extend(["-c", str(args.count)])
    command.append(build_filter(args))
    return command


def run_local_capture(args):
    output = os.path.abspath(os.path.expanduser(args.output or default_output_path("motion_host")))
    command = build_tcpdump_args(args, output)
    print("capture output: %s" % output)
    print("command:")
    print("  " + shell_join(command))
    if args.dry_run:
        return 0
    os.makedirs(os.path.dirname(output), exist_ok=True)
    process = subprocess.Popen(command)

    def stop(_signum, _frame):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if args.duration > 0 and args.count <= 0:
        try:
            return process.wait(timeout=args.duration)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
    return process.wait()


def run_remote_capture(args):
    if not args.ssh:
        args.ssh = "%s@%s" % (args.ssh_user, args.motion_host)
    output = os.path.abspath(os.path.expanduser(args.output or default_output_path("motion_host_remote")))
    remote_args = build_tcpdump_args(args, "-")
    if remote_args[0] == "sudo":
        if args.sudo_with_password or args.sudo_password:
            remote_args = ["sudo", "-S", "-p", ""] + remote_args[1:]
        else:
            remote_args = ["sudo", "-n"] + remote_args[1:]
    ssh_command = ["ssh", args.ssh, shell_join(remote_args)]
    env = None
    if args.sshpass:
        if not args.ssh_password and args.ask_password:
            args.ssh_password = getpass.getpass("SSH password for %s: " % args.ssh)
        if args.ssh_password:
            ssh_command = ["sshpass", "-e"] + ssh_command
            env = os.environ.copy()
            env["SSHPASS"] = args.ssh_password
    print("local output: %s" % output)
    print("remote command:")
    print("  " + shell_join(redact_command_for_print(ssh_command)))
    if args.dry_run:
        return 0
    sudo_stdin = None
    if args.sudo and (args.sudo_with_password or args.sudo_password):
        if not args.sudo_password and args.ask_password:
            args.sudo_password = getpass.getpass("sudo password on %s: " % args.ssh)
        if not args.sudo_password:
            print("sudo password is required; set motion_host_sudo_password or sudo_password_same_as_ssh", file=sys.stderr)
            return 2
        sudo_stdin = (args.sudo_password + "\n").encode("utf-8")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "wb") as f:
        process = subprocess.Popen(ssh_command, stdin=subprocess.PIPE if sudo_stdin else None, stdout=f, env=env)
        if sudo_stdin:
            process.stdin.write(sudo_stdin)
            process.stdin.flush()
            process.stdin.close()

        def stop(_signum, _frame):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        if args.duration > 0 and args.count <= 0:
            try:
                return process.wait(timeout=args.duration)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
        return process.wait()


def summarize_pcap(args):
    command = ["tcpdump", "-nn", "-tttt", "-r", args.pcap]
    if args.count > 0:
        command.extend(["-c", str(args.count)])
    print("command:")
    print("  " + shell_join(command))
    if args.dry_run:
        return 0
    return subprocess.call(command)


def read_pcap_records(path):
    with open(path, "rb") as f:
        header = f.read(24)
        if len(header) != 24:
            raise ValueError("pcap global header is too short")
        magic = header[:4]
        if magic == b"\xd4\xc3\xb2\xa1":
            endian = "<"
        elif magic == b"\xa1\xb2\xc3\xd4":
            endian = ">"
        else:
            raise ValueError("only classic pcap is supported; pcapng is not supported")
        _magic, _vmaj, _vmin, _tz, _sig, _snaplen, linktype = struct.unpack(endian + "IHHIIII", header)
        while True:
            rec_header = f.read(16)
            if not rec_header:
                break
            if len(rec_header) != 16:
                break
            ts_sec, ts_usec, incl_len, orig_len = struct.unpack(endian + "IIII", rec_header)
            data = f.read(incl_len)
            if len(data) != incl_len:
                break
            yield endian, linktype, ts_sec + ts_usec / 1000000.0, data, orig_len


def parse_ipv4_udp(linktype, frame):
    if linktype == 1:
        if len(frame) < 14 or frame[12:14] != b"\x08\x00":
            return None
        ip = frame[14:]
    elif linktype == 113:
        if len(frame) < 16 or frame[14:16] != b"\x08\x00":
            return None
        ip = frame[16:]
    else:
        return None
    if len(ip) < 20:
        return None
    ihl = (ip[0] & 0x0F) * 4
    if len(ip) < ihl + 8 or ip[9] != 17:
        return None
    src = ".".join(str(b) for b in ip[12:16])
    dst = ".".join(str(b) for b in ip[16:20])
    udp = ip[ihl:]
    src_port, dst_port, udp_len, _checksum = struct.unpack("!HHHH", udp[:8])
    payload = udp[8 : max(8, udp_len)]
    return src, src_port, dst, dst_port, payload


def decode_payload(payload):
    if len(payload) == 12:
        code, value, cmd_type = struct.unpack("<iii", payload)
        return "SimpleCMD code=0x%08X(%d) value=%d type=%d" % (code & 0xFFFFFFFF, code, value, cmd_type)
    if len(payload) == 20:
        code, value, cmd_type, data = struct.unpack("<iiid", payload)
        return "ComplexCMD code=0x%08X(%d) value=%d type=%d data=%.6f" % (
            code & 0xFFFFFFFF,
            code,
            value,
            cmd_type,
            data,
        )
    if len(payload) == 42 and payload[:2] == b"\x55\x66":
        ctrl = payload[2]
        data_len, seq = struct.unpack("<HH", payload[3:7])
        controller_id = payload[7]
        checksum = struct.unpack("<H", payload[8:10])[0]
        channels = struct.unpack("<16h", payload[10:42])
        return (
            "JoystickChannelFrame ctrl=%d len=%d seq=%d id=%d checksum=0x%04X "
            "left=(%d,%d) right=(%d,%d)"
            % (ctrl, data_len, seq, controller_id, checksum, channels[10], channels[11], channels[12], channels[13])
        )
    return "udp_payload len=%d hex=%s" % (len(payload), payload[:32].hex())


def decode_pcap(args):
    count = 0
    for _endian, linktype, ts, frame, _orig_len in read_pcap_records(args.pcap):
        udp = parse_ipv4_udp(linktype, frame)
        if udp is None:
            continue
        src, src_port, dst, dst_port, payload = udp
        if args.port and src_port not in args.port and dst_port not in args.port:
            continue
        filter_hosts = args.filter_host[:] or ([args.motion_host] if args.motion_host else [])
        if filter_hosts and not any(host in (src, dst) for host in filter_hosts):
            continue
        print(
            "%s %s:%d -> %s:%d %s"
            % (
                _dt.datetime.fromtimestamp(ts).isoformat(timespec="milliseconds"),
                src,
                src_port,
                dst,
                dst_port,
                decode_payload(payload),
            )
        )
        count += 1
        if args.count > 0 and count >= args.count:
            break
    if count == 0:
        print("no decodable UDP records found; try `summarize` or capture with classic pcap instead of pcapng")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Passive tcpdump helper for Lite3 motion-host traffic. It never sends robot commands."
    )
    parser.add_argument(
        "mode",
        choices=["capture", "remote-capture", "summarize", "decode"],
        help="capture locally, capture over ssh, summarize pcap with tcpdump, or decode known UDP payloads.",
    )
    parser.add_argument("--motion-host", default="192.168.1.120")
    parser.add_argument("--motion-host-p2p", default="192.168.2.1")
    parser.add_argument("--handheld-ip", default="")
    parser.add_argument("--tablet-ip", default="")
    parser.add_argument(
        "--filter-host",
        action="append",
        default=[],
        help="Primary tcpdump host filter. If omitted, defaults to --motion-host.",
    )
    parser.add_argument("--host", action="append", default=[], help="Extra host filter. Can be repeated.")
    parser.add_argument("--port", action="append", type=int, default=None)
    parser.add_argument("--all-ports", action="store_true", help="Do not restrict capture by port.")
    parser.add_argument("--interface", default="any")
    parser.add_argument("--snaplen", type=int, default=0)
    parser.add_argument("--duration", type=int, default=60, help="Capture seconds. 0 means until Ctrl-C.")
    parser.add_argument("--count", type=int, default=0, help="Packet count limit for capture/summarize/decode.")
    parser.add_argument("--output", default="", help="Output pcap path.")
    parser.add_argument("--pcap", default="", help="Input pcap path for summarize/decode.")
    parser.add_argument("--ssh", default="", help="user@host for remote-capture. Default: ysc@<motion-host>.")
    parser.add_argument("--ssh-user", default="ysc", help="Default SSH user when --ssh is not set.")
    parser.add_argument(
        "--private-config",
        default=os.path.join(os.path.dirname(__file__), DEFAULT_PRIVATE_CONFIG),
        help="Ignored local YAML file with private robot access settings.",
    )
    parser.add_argument("--ssh-password", default="", help="SSH password. Prefer private config; not printed.")
    parser.add_argument("--sshpass", action="store_true", default=None, help="Use sshpass -e for password based SSH.")
    parser.add_argument("--ask-password", action="store_true", help="Prompt for SSH password if needed.")
    parser.add_argument("--sudo-password", default="", help="Remote sudo password. Prefer private config; not printed.")
    parser.add_argument("--sudo-with-password", action="store_true", default=None, help="Use sudo -S for remote tcpdump.")
    parser.add_argument("--sudo", action="store_true", default=None)
    parser.add_argument("--no-sudo", action="store_false", dest="sudo")
    parser.add_argument("--udp-only", action="store_true")
    parser.add_argument("--tcp-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    apply_private_config(args)
    if args.sshpass is None:
        args.sshpass = False
    if args.sudo is None:
        args.sudo = False
    if args.sudo_with_password is None:
        args.sudo_with_password = False
    if args.port is None:
        args.port = DEFAULT_PORTS[:]
    if args.udp_only and args.tcp_only:
        parser.error("--udp-only and --tcp-only cannot be used together")
    if args.mode in ("summarize", "decode") and not args.pcap:
        parser.error("--pcap is required for summarize/decode")
    return args


def main():
    args = parse_args()
    if args.mode == "capture":
        return run_local_capture(args)
    if args.mode == "remote-capture":
        return run_remote_capture(args)
    if args.mode == "summarize":
        return summarize_pcap(args)
    if args.mode == "decode":
        return decode_pcap(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
