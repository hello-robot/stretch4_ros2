#!/usr/bin/env python3
"""
stretch_lidar_check — Read-only JT128 head lidar PTP and config verification.

Checks return mode, point-cloud filter, PTP lock offset, locked status,
and 30s jitter per lidar.
"""

import argparse
import json
import os
import statistics
import sys
import time

from stretch4_pyhesai_wrapper.ptc_client import (
    FILTER_NAMES,
    FILTER_STRONG,
    LEFT_LIDAR_IP,
    PTP_LOCK_OFFSET_US,
    PTP_STATUS_LOCKED,
    RETURN_MODE_LAST_AND_STRONGEST,
    RETURN_MODE_NAMES,
    RIGHT_LIDAR_IP,
    ULTRA_PRECISE_NAMES,
    HesaiPtcError,
    get_lidar_ptp_status,
    get_point_cloud_config,
    get_ptp_lock_offset_us,
    get_return_mode,
    ptc_reachable,
    sample_ptp_offset_us,
)

JITTER_RATE_HZ = 4
DEFAULT_DURATION_S = 30

LIDARS = (
    ('left', LEFT_LIDAR_IP),
    ('right', RIGHT_LIDAR_IP),
)

PTP_GM_HINT = (
    'Ensure NUC PTP grandmaster is running (lidar-ptp4l + lidar-phc2sys).'
)


def _ptp_percentile(sorted_values, pct):
    """Return the pct-th percentile (0-100) from a sorted list."""
    if not sorted_values:
        raise ValueError('empty sample set')
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = int(len(sorted_values) * pct / 100.0)
    index = min(index, len(sorted_values) - 1)
    return sorted_values[index]


def sample_jitter(ip, duration_s, verbose=False):
    """
    Poll PTP offset for duration_s at JITTER_RATE_HZ; return stats dict.
    """
    interval_s = 1.0 / JITTER_RATE_HZ
    samples_us = []
    errors = []
    start = time.time()

    while (time.time() - start) < duration_s:
        try:
            offset_us, _diag = sample_ptp_offset_us(ip)
            samples_us.append(offset_us)
        except Exception as exc:
            errors.append(str(exc))
            if verbose:
                print(
                    '    jitter sample error: {}'.format(exc),
                    file=sys.stderr,
                )
        time.sleep(interval_s)

    if not samples_us:
        return {
            'count': 0,
            'p95_us': None,
            'median_us': None,
            'max_us': None,
            'errors': errors,
        }

    sorted_us = sorted(samples_us)
    return {
        'count': len(samples_us),
        'p95_us': _ptp_percentile(sorted_us, 95),
        'median_us': statistics.median(samples_us),
        'max_us': sorted_us[-1],
        'errors': errors,
    }


def _status_line(label, passed, detail=''):
    """Format a check result line."""
    status = 'PASS' if passed else 'FAIL'
    width = 30
    dots = '.' * max(1, width - len(label))
    line = '  {} {} {}'.format(label, dots, status)
    if detail:
        line += ' ({})'.format(detail)
    return line


def check_lidar(side, ip, duration_s, verbose=False):
    """
    Run all read-only checks for one lidar; return result dict.
    """
    result = {
        'side': side,
        'ip': ip,
        'checks': {},
        'passed': True,
    }

    # PTC reachable
    reachable = ptc_reachable(ip)
    result['checks']['ptc_reachable'] = {
        'passed': reachable,
        'value': reachable,
    }
    if not reachable:
        result['passed'] = False
        return result

    # Return mode
    try:
        mode = get_return_mode(ip)
        mode_ok = mode == RETURN_MODE_LAST_AND_STRONGEST
        result['checks']['return_mode'] = {
            'passed': mode_ok,
            'value': mode,
            'expected': RETURN_MODE_LAST_AND_STRONGEST,
            'name': RETURN_MODE_NAMES.get(mode, 'unknown'),
        }
        if not mode_ok:
            result['passed'] = False
    except HesaiPtcError as exc:
        result['checks']['return_mode'] = {'passed': False, 'error': str(exc)}
        result['passed'] = False

    # Point-cloud filter
    try:
        ultra_precise, filt = get_point_cloud_config(ip)
        filter_ok = filt == FILTER_STRONG
        result['checks']['point_cloud_filter'] = {
            'passed': filter_ok,
            'value': filt,
            'expected': FILTER_STRONG,
            'name': FILTER_NAMES.get(filt, 'unknown'),
            'ultra_precise': ultra_precise,
            'ultra_precise_name': ULTRA_PRECISE_NAMES.get(
                ultra_precise, 'unknown',
            ),
        }
        if not filter_ok:
            result['passed'] = False
    except HesaiPtcError as exc:
        result['checks']['point_cloud_filter'] = {
            'passed': False, 'error': str(exc),
        }
        result['passed'] = False

    # Lock offset
    try:
        offset_us = get_ptp_lock_offset_us(ip)
        offset_ok = offset_us == PTP_LOCK_OFFSET_US
        result['checks']['lock_offset_us'] = {
            'passed': offset_ok,
            'value': offset_us,
            'expected': PTP_LOCK_OFFSET_US,
        }
        if not offset_ok:
            result['passed'] = False
    except HesaiPtcError as exc:
        result['checks']['lock_offset_us'] = {
            'passed': False, 'error': str(exc),
        }
        result['passed'] = False

    # PTP status (strict: locked only, single read)
    try:
        ptp = get_lidar_ptp_status(ip)
        status = ptp['ptp_status']
        status_name = ptp['ptp_status_name']
        status_ok = status == PTP_STATUS_LOCKED
        result['checks']['ptp_status'] = {
            'passed': status_ok,
            'value': status_name,
            'status_code': status,
            'expected': 'locked',
        }
        if not status_ok:
            result['passed'] = False
            if status_name == 'free_run':
                result['checks']['ptp_status']['hint'] = PTP_GM_HINT
    except HesaiPtcError as exc:
        result['checks']['ptp_status'] = {'passed': False, 'error': str(exc)}
        result['passed'] = False

    # Jitter
    jitter = sample_jitter(ip, duration_s, verbose=verbose)
    jitter_ok = (
        jitter['count'] > 0
        and jitter['p95_us'] is not None
        and jitter['p95_us'] <= PTP_LOCK_OFFSET_US
    )
    result['checks']['jitter'] = {
        'passed': jitter_ok,
        'duration_s': duration_s,
        'rate_hz': JITTER_RATE_HZ,
        'count': jitter['count'],
        'p95_us': jitter['p95_us'],
        'median_us': jitter['median_us'],
        'max_us': jitter['max_us'],
        'threshold_us': PTP_LOCK_OFFSET_US,
    }
    if jitter.get('errors'):
        result['checks']['jitter']['sample_errors'] = jitter['errors']
    if not jitter_ok:
        result['passed'] = False

    return result


def print_lidar_report(result, duration_s):
    """Print human-readable report for one lidar."""
    side = result['side'].upper()
    ip = result['ip']
    print('\n{}  ({})'.format(side, ip))

    checks = result['checks']

    reachable = checks.get('ptc_reachable', {})
    print(_status_line('PTC reachable', reachable.get('passed', False)))

    if not reachable.get('passed'):
        return

    rm = checks.get('return_mode', {})
    if 'error' in rm:
        print(_status_line('Return mode (Last+Strongest)', False, rm['error']))
    else:
        detail = '{} ({})'.format(rm.get('value'), rm.get('name', ''))
        print(_status_line(
            'Return mode (Last+Strongest)', rm.get('passed'), detail,
        ))

    pf = checks.get('point_cloud_filter', {})
    if 'error' in pf:
        print(_status_line('Point-cloud filter (Strong)', False, pf['error']))
    else:
        detail = '{} ({})'.format(pf.get('value'), pf.get('name', ''))
        print(_status_line(
            'Point-cloud filter (Strong)', pf.get('passed'), detail,
        ))

    lo = checks.get('lock_offset_us', {})
    if 'error' in lo:
        print(_status_line('Lock offset', False, lo['error']))
    else:
        print(_status_line(
            'Lock offset', lo.get('passed'),
            '{} µs'.format(lo.get('value')),
        ))

    ps = checks.get('ptp_status', {})
    if 'error' in ps:
        print(_status_line('PTP status', False, ps['error']))
    else:
        print(_status_line('PTP status', ps.get('passed'), ps.get('value')))
        if ps.get('hint'):
            print('    hint: {}'.format(ps['hint']))

    jt = checks.get('jitter', {})
    if jt.get('count', 0) == 0:
        print(_status_line(
            'Jitter {}s'.format(int(duration_s)), False, 'no samples',
        ))
    else:
        detail = 'p95={:.1f} µs  median={:.1f} µs  max={:.1f} µs'.format(
            jt['p95_us'], jt['median_us'], jt['max_us'],
        )
        print(_status_line(
            'Jitter {}s'.format(int(duration_s)), jt.get('passed'), detail,
        ))


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            'Read-only check of JT128 head lidar return mode, '
            'point-cloud filter, PTP lock offset, locked status, and jitter.'
        ),
        epilog=PTP_GM_HINT,
    )
    parser.add_argument(
        '--left', action='store_true', help='Check left lidar only.',
    )
    parser.add_argument(
        '--right', action='store_true', help='Check right lidar only.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=DEFAULT_DURATION_S,
        help='Jitter sample duration in seconds (default: {}).'.format(
            DEFAULT_DURATION_S,
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print machine-readable JSON summary.',
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print per-sample jitter errors to stderr.',
    )
    return parser.parse_args(argv)


def main(argv=None):
    """Entry point for stretch_lidar_check."""
    args = parse_args(argv)

    if args.left and args.right:
        print('Use only one of --left or --right.', file=sys.stderr)
        return 1

    if args.left:
        targets = [('left', LEFT_LIDAR_IP)]
    elif args.right:
        targets = [('right', RIGHT_LIDAR_IP)]
    else:
        targets = list(LIDARS)

    hostname = os.uname().nodename
    results = []
    all_passed = True

    if not args.json:
        print('stretch_lidar_check — {}'.format(hostname))

    for side, ip in targets:
        result = check_lidar(
            side, ip, args.duration, verbose=args.verbose,
        )
        results.append(result)
        if not result['passed']:
            all_passed = False
        if not args.json:
            print_lidar_report(result, args.duration)

    summary = {
        'hostname': hostname,
        'duration_s': args.duration,
        'jitter_rate_hz': JITTER_RATE_HZ,
        'overall_passed': all_passed,
        'lidars': results,
    }

    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print('\nOVERALL: {}'.format('PASS' if all_passed else 'FAIL'))

    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
