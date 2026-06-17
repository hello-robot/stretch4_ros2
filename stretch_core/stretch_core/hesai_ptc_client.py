#!/usr/bin/env python3
"""
JT128 Hesai PTC (TCP) read-only client for stretch_lidar_check.

Validated against stretch_production_tools_ii/hesai_lidar_utils.py.
Keep in sync with that module.
"""

import socket
import struct

PTC_PORT = 9347
PTC_MAGIC_1 = 0x47
PTC_MAGIC_2 = 0x74

PTC_HEADER_FMT = '>BBBB I'
PTC_HEADER_SIZE = struct.calcsize(PTC_HEADER_FMT)

CMD_GET_PTP_DIAGNOSTICS = 0x06
PTP_DIAGNOSTICS_SUBCOMMAND = 1
PTP_DIAGNOSTICS_PAYLOAD_LEN = 24
CMD_GET_LIDAR_STATUS = 0x09
CMD_GET_CONFIG_INFO = 0x08
CMD_GET_PTP_LOCK_OFFSET = 0x3A

LEFT_LIDAR_IP = '192.168.1.202'
RIGHT_LIDAR_IP = '192.168.1.201'

CONFIG_RETURN_MODE_OFFSET = 32
LIDAR_STATUS_PTP_OFFSET = 52
LIDAR_STATUS_MIN_LEN = 53

RETURN_MODE_LAST_AND_STRONGEST = 2
PTP_LOCK_OFFSET_US = 350

RETURN_MODE_NAMES = {
    0: 'last',
    1: 'strongest',
    2: 'last_and_strongest',
    3: 'first',
    4: 'last_and_first',
    5: 'first_and_strongest',
}

PTP_STATUS_FREE_RUN = 0
PTP_STATUS_TRACKING = 1
PTP_STATUS_LOCKED = 2
PTP_STATUS_FROZEN = 3

PTP_STATUS_NAMES = {
    PTP_STATUS_FREE_RUN: 'free_run',
    PTP_STATUS_TRACKING: 'tracking',
    PTP_STATUS_LOCKED: 'locked',
    PTP_STATUS_FROZEN: 'frozen',
}


class HesaiPtcError(Exception):
    """Raised when a PTC command fails."""


def _recv_exact(sock, nbytes):
    """Read exactly nbytes from socket or raise."""
    chunks = []
    remaining = nbytes
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise HesaiPtcError(
                'Connection closed while reading {} bytes'.format(nbytes)
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def ptc_query(ip, cmd, payload=b'', timeout=2.0):
    """
    Send a PTC command and return the response payload bytes.

    Raises HesaiPtcError on TCP or protocol errors.
    """
    header = struct.pack(
        PTC_HEADER_FMT,
        PTC_MAGIC_1,
        PTC_MAGIC_2,
        cmd,
        0x00,
        len(payload),
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect((ip, PTC_PORT))
        sock.sendall(header + payload)

        resp_header = _recv_exact(sock, PTC_HEADER_SIZE)
        magic1, magic2, resp_cmd, ret_code, payload_len = struct.unpack(
            PTC_HEADER_FMT, resp_header
        )
        if magic1 != PTC_MAGIC_1 or magic2 != PTC_MAGIC_2:
            raise HesaiPtcError(
                'Bad response magic 0x{:02X} 0x{:02X}'.format(magic1, magic2)
            )
        if ret_code != 0:
            raise HesaiPtcError(
                'PTC cmd 0x{:02X} failed with ret_code={}'.format(
                    cmd, ret_code,
                )
            )
        if resp_cmd != cmd:
            raise HesaiPtcError(
                'PTC response cmd 0x{:02X} != request 0x{:02X}'.format(
                    resp_cmd, cmd
                )
            )

        if payload_len == 0:
            return b''
        return _recv_exact(sock, payload_len)


def ptc_reachable(ip, timeout=2.0):
    """Return True if PTC TCP port accepts a connection."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect((ip, PTC_PORT))
        return True
    except OSError:
        return False


def get_return_mode(ip, timeout=2.0):
    """
    Read return mode from GET_CONFIG_INFO (0x08), byte index 32.
    """
    payload = ptc_query(ip, CMD_GET_CONFIG_INFO, timeout=timeout)
    if len(payload) <= CONFIG_RETURN_MODE_OFFSET:
        raise HesaiPtcError(
            'GET_CONFIG_INFO payload too short for return_mode '
            '(len={})'.format(len(payload))
        )
    return payload[CONFIG_RETURN_MODE_OFFSET]


def get_ptp_lock_offset_us(ip, timeout=2.0):
    """
    Read PTP lock offset threshold in microseconds (PTC 0x3A, uint16 BE).
    """
    payload = ptc_query(ip, CMD_GET_PTP_LOCK_OFFSET, timeout=timeout)
    if len(payload) != 2:
        raise HesaiPtcError(
            'GET_PTP_LOCK_OFFSET expected 2 bytes, got {}'.format(len(payload))
        )
    return struct.unpack('>H', payload)[0]


def get_lidar_ptp_status(ip, timeout=2.0):
    """
    Read PTP status from GET_LIDAR_STATUS (0x09), byte index 52.

    Per JT128 TCP API: 0=free_run, 1=tracking, 2=locked, 3=frozen.
    """
    payload = ptc_query(ip, CMD_GET_LIDAR_STATUS, timeout=timeout)
    if len(payload) < LIDAR_STATUS_MIN_LEN:
        raise HesaiPtcError(
            'GET_LIDAR_STATUS payload too short for ptp_status '
            '(len={})'.format(len(payload))
        )
    status = payload[LIDAR_STATUS_PTP_OFFSET]
    return {
        'ptp_status': status,
        'ptp_status_name': PTP_STATUS_NAMES.get(status, 'unknown'),
    }


def get_ptp_diagnostics(ip, timeout=2.0):
    """
    Query PTP master offset (0x06 subcommand 1) and status (0x09 byte 52).
    """
    payload = ptc_query(
        ip,
        CMD_GET_PTP_DIAGNOSTICS,
        struct.pack('>B', PTP_DIAGNOSTICS_SUBCOMMAND),
        timeout=timeout,
    )
    if len(payload) != PTP_DIAGNOSTICS_PAYLOAD_LEN:
        raise HesaiPtcError(
            'GET_PTP_DIAGNOSTICS expected {} bytes, got {}'.format(
                PTP_DIAGNOSTICS_PAYLOAD_LEN, len(payload)
            )
        )

    offset_ns = struct.unpack('>q', payload[:8])[0]
    status_info = get_lidar_ptp_status(ip, timeout=timeout)
    return {
        'offset_ns': offset_ns,
        'offset_us': abs(offset_ns / 1000.0),
        'ptp_status': status_info['ptp_status'],
        'ptp_status_name': status_info['ptp_status_name'],
    }


def sample_ptp_offset_us(ip, timeout=2.0):
    """Sample one |PTP offset| reading in microseconds."""
    diag = get_ptp_diagnostics(ip, timeout=timeout)
    return diag['offset_us'], diag
