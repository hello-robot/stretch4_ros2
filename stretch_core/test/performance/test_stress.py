import pytest
import rclpy
import multiprocessing
import threading
import concurrent.futures
import time
import math
import os
import tempfile
import socket

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.fjt_client import FJTClient


def cpu_stressor():
    """Burns CPU cycles to simulate high load"""
    while True:
        math.factorial(5000)


def network_stressor():
    """Generates heavy loopback UDP traffic to simulate network contention"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = os.urandom(8192) # 8KB packets
    while True:
        try:
            sock.sendto(data, ('127.0.0.1', 9999))
        except OSError:
            pass


def usb_io_stressor():
    """Generates heavy file I/O to simulate bus/system I/O contention"""
    with tempfile.NamedTemporaryFile() as f:
        data = os.urandom(1024 * 1024 * 10) # 10MB chunks
        while True:
            f.write(data)
            f.flush()
            f.seek(0)


@pytest.fixture
def action_client():
    rclpy.init()
    node = FJTClient('stress_action_client')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def run_trajectory_under_load(benchmark, client, mode):
    """Common logic for benchmarking a trajectory while under load."""
    def run_traj():
        client.mode = 'position'
        q = {'lift_joint': 0.5}
        result = client.move_to_configuration(q, blocking=True)
        assert result.status == 4 # Succeeded
        
        # Move back
        q = {'lift_joint': 0.2}
        result = client.move_to_configuration(q, blocking=True)
        assert result.status == 4
        
    benchmark(run_traj)


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_stress_cpu_load(benchmark, action_client):
    """
    Evaluates trajectory execution success and latency while simulating
    artificial high CPU load on the system.
    """
    # Start stressors based on core count
    cores_to_use = max(1, multiprocessing.cpu_count() - 1)
    processes = []
    
    try:
        for _ in range(cores_to_use):
            p = multiprocessing.Process(target=cpu_stressor)
            p.daemon = True
            p.start()
            processes.append(p)
            
        time.sleep(1.0) # Let load build up
        run_trajectory_under_load(benchmark, action_client, 'cpu_stress')
        
    finally:
        for p in processes:
            p.terminate()
            p.join()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_stress_network_traffic(benchmark, action_client):
    """
    Evaluates latency and success rates while simulating background
    network activity on loopback, challenging DDS discovery/traffic.
    """
    p = multiprocessing.Process(target=network_stressor)
    p.daemon = True
    
    try:
        p.start()
        time.sleep(1.0)
        run_trajectory_under_load(benchmark, action_client, 'network_stress')
    finally:
        p.terminate()
        p.join()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_stress_usb_traffic(benchmark, action_client):
    """
    Simulates heavy background file I/O to approximate bus contention
    while verifying stretch_driver communication stability.
    """
    p = multiprocessing.Process(target=usb_io_stressor)
    p.daemon = True
    
    try:
        p.start()
        time.sleep(1.0)
        run_trajectory_under_load(benchmark, action_client, 'usb_io_stress')
    finally:
        p.terminate()
        p.join()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_stress_spam_fjt():
    """
    The action server is configured with a QoS depth of 50 for its goal service, meaning
    it can handle up to 50 concurrent goal requests. Additional requests are dropped.
    This test verifies that the server successfully handles 50 concurrent requests.
    """
    if not rclpy.ok():
        rclpy.init()
    
    num_clients = 50
    clients = []
    
    try:
        # Create multiple clients
        for i in range(num_clients):
            # Give each client a unique name
            c = FJTClient(f'reentrancy_client_{i}')
            clients.append(c)

        # Function to execute goal
        def send_goal_wrapper(client_idx, client):
            # Use unique joint name as identifier
            joint_name = f'client_{client_idx}_thread'
            try:
                # move_to_configuration is blocking by default and handles waiting for result
                # It sends a goal with position 0.0 for the named joint
                client.move_to_configuration({joint_name: 0.0}, blocking=True)
                return True, f"Client {client_idx}: Success"
            except Exception as e:
                return False, f"Client {client_idx}: Failed - {e}"

        results = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_clients) as executor:
            # Submit all tasks
            futures = [executor.submit(send_goal_wrapper, i, clients[i]) for i in range(num_clients)]
            
            # Wait for all to complete
            for future in concurrent.futures.as_completed(futures):
                results.append(future.result())
        
        # Verify results
        success_count = 0
        for success, msg in results:
            if success:
                success_count += 1
                
        assert success_count == num_clients, f"Expected {num_clients} successes, got {success_count}"
        
    finally:
        for c in clients:
            c.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
