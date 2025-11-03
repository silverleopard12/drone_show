#!/usr/bin/env python3
"""
Fully Automated Formation Pipeline
===================================
A폴더(시작) + B폴더(끝) + 드론 수 → 자동으로 궤적 txt 파일 생성

Usage:
    # Preset 포메이션
    python3 auto_formation_pipeline.py --num_drones 36 --output_folder my_trajectories

    # Custom 포메이션
    python3 auto_formation_pipeline.py \
        --current formations/start.txt \
        --target formations/end.txt \
        --output_folder my_trajectories

    # 실행 시간 제한 (60초 후 자동 종료)
    python3 auto_formation_pipeline.py --num_drones 36 --timeout 60
"""

import os
import sys
import argparse
import subprocess
import time
import signal
import threading
from pathlib import Path
from datetime import datetime

# Add Allocation directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Allocation'))

from formations_large import get_formations
from Fair_Hungarian_Allocator import HungarianAllocator


class AutoFormationPipeline:
    """완전 자동화 포메이션 파이프라인"""

    def __init__(self, workspace_root=None):
        if workspace_root is None:
            self.workspace_root = Path(__file__).parent.parent
        else:
            self.workspace_root = Path(workspace_root)

        self.scripts_dir = self.workspace_root / "scripts"
        self.src_dir = self.workspace_root / "src"
        self.setup_bash = self.workspace_root / "install" / "setup.bash"

        # Process tracking
        self.sim_process = None
        self.recorder_process = None
        self.mission_completed = False

        print(f"🚀 Auto Formation Pipeline")
        print(f"📁 Workspace: {self.workspace_root}")
        print()

    def load_formation_from_file(self, filepath):
        """파일에서 포메이션 로드 (x,y,z per line)"""
        formation = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    formation.append([x, y, z])
        return formation

    def load_formations(self, num_drones=None, current_file=None, target_file=None):
        """포메이션 로드"""
        print("="*70)
        print("📥 STEP 1: Loading Formations")
        print("="*70)

        if current_file and target_file:
            formation_a = self.load_formation_from_file(current_file)
            formation_b = self.load_formation_from_file(target_file)
            num_drones = len(formation_a)
            print(f"✓ Start formation: {num_drones} drones from {current_file}")
            print(f"✓ End formation: {num_drones} drones from {target_file}")

            if len(formation_a) != len(formation_b):
                raise ValueError(f"Formation size mismatch: {len(formation_a)} vs {len(formation_b)}")

        elif num_drones:
            formation_a, formation_b = get_formations(num_drones)
            print(f"✓ Start formation: {num_drones} drones (Preset Grid)")
            print(f"✓ End formation: {num_drones} drones (Preset Triangle)")

        else:
            raise ValueError("Must specify either num_drones or both current_file and target_file")

        print()
        return formation_a, formation_b, num_drones

    def run_hungarian_allocation(self, formation_a, formation_b):
        """Fair Hungarian Allocation"""
        print("="*70)
        print("🎯 STEP 2: Fair Hungarian Allocation")
        print("="*70)

        drones = [[pos[0], pos[1], pos[2]] for pos in formation_a]
        goals = [[pos[0], pos[1], pos[2]] for pos in formation_b]

        allocator = HungarianAllocator(max_iterations=50)
        assignment = allocator.allocate_fair(drones, goals)

        print("✓ Allocation completed")
        print()
        return assignment

    def generate_launch_file(self, formation_a, formation_b, assignment, num_drones):
        """Launch 파일 생성"""
        print("="*70)
        print("📝 STEP 3: Generating Launch File")
        print("="*70)

        # drone_configs 생성
        drone_configs = []
        for drone_id in range(num_drones):
            goal_id = assignment[drone_id]
            init_pos = formation_a[drone_id]
            target_pos = formation_b[goal_id]

            drone_configs.append({
                'drone_id': drone_id,
                'init_x': init_pos[0],
                'init_y': init_pos[1],
                'init_z': init_pos[2],
                'target_x': target_pos[0],
                'target_y': target_pos[1],
                'target_z': target_pos[2]
            })

        # Launch 파일 내용
        launch_content = f"""import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Map parameters
    map_size_x = LaunchConfiguration('map_size_x', default=10.0)
    map_size_y = LaunchConfiguration('map_size_y', default=100.0)
    map_size_z = LaunchConfiguration('map_size_z', default=50.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    # Drone configurations - {num_drones} drones (Auto-generated)
    drone_configs = [
"""

        for config in drone_configs:
            launch_content += f"        {config},\n"

        launch_content += f"""    ]

    drone_nodes = []
    for config in drone_configs:
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim_minimal.launch.py')),
            launch_arguments={{
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z']),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic
            }}.items()
        )
        drone_nodes.append(drone_launch)

    # Swarm Synchronizer
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('ego_planner')))))
    swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer.py')
    swarm_synchronizer = ExecuteProcess(
        cmd=['python3', swarm_sync_script, '--ros-args', '-p', 'num_drones:={num_drones}', '-p', 'wait_time:=5.0'],
        output='screen'
    )

    # Mission Timer
    mission_timer_script = os.path.join(project_root, 'scripts', 'mission_timer.py')
    timer_cmd = ['python3', mission_timer_script, '--ros-args', '-p', 'num_drones:={num_drones}', '-p', 'arrival_threshold:=0.5']
    for config in drone_configs:
        drone_id = config['drone_id']
        timer_cmd.extend([
            '-p', f'drone_{{drone_id}}_target_x:={{config["target_x"]}}',
            '-p', f'drone_{{drone_id}}_target_y:={{config["target_y"]}}',
            '-p', f'drone_{{drone_id}}_target_z:={{config["target_z"]}}'
        ])
    mission_timer = ExecuteProcess(cmd=timer_cmd, output='screen')

    ld = LaunchDescription()
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld
"""

        # 파일 저장
        launch_file = self.src_dir / "planner" / "plan_manage" / "launch" / "auto_generated_swarm.launch.py"
        with open(launch_file, 'w') as f:
            f.write(launch_content)

        print(f"✓ Launch file: {launch_file}")
        print()
        return launch_file

    def monitor_mission_completion(self, timeout):
        """미션 완료 모니터링 (timeout 후 자동 종료)"""
        print(f"⏱️  Timeout: {timeout} seconds")
        time.sleep(timeout)

        print("\n" + "="*70)
        print("⏰ Timeout reached - Stopping simulation")
        print("="*70)

        self.mission_completed = True
        self.stop_all_processes()

    def stop_all_processes(self):
        """모든 프로세스 종료"""
        if self.sim_process and self.sim_process.poll() is None:
            print("🛑 Stopping simulation...")
            self.sim_process.send_signal(signal.SIGINT)
            time.sleep(2)
            if self.sim_process.poll() is None:
                self.sim_process.kill()

        if self.recorder_process and self.recorder_process.poll() is None:
            print("🛑 Stopping recorder...")
            self.recorder_process.send_signal(signal.SIGINT)
            time.sleep(2)
            if self.recorder_process.poll() is None:
                self.recorder_process.kill()

    def run_full_pipeline(self, num_drones=None, current_file=None, target_file=None,
                         output_folder="trajectories", sampling_dt=0.05, timeout=120):
        """완전 자동화 파이프라인"""

        print("\n" + "="*70)
        print("🤖 FULLY AUTOMATED FORMATION PIPELINE")
        print("="*70)
        print(f"📊 Output folder: {output_folder}")
        print(f"⏱️  Timeout: {timeout}s")
        print(f"📏 Sampling dt: {sampling_dt}s")
        print("="*70)
        print()

        try:
            # Step 1-3: Planning
            formation_a, formation_b, num_drones = self.load_formations(num_drones, current_file, target_file)
            assignment = self.run_hungarian_allocation(formation_a, formation_b)
            launch_file = self.generate_launch_file(formation_a, formation_b, assignment, num_drones)

            # Step 4: Run Simulation
            print("="*70)
            print("🚁 STEP 4: Running Ego-Swarm Simulation")
            print("="*70)
            print("Starting simulation and trajectory recorder...")
            print()

            # Source ROS2 and run simulation
            sim_cmd = f"source {self.setup_bash} && ros2 launch {launch_file}"
            self.sim_process = subprocess.Popen(
                sim_cmd,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            # Wait for simulation to initialize
            time.sleep(10)

            # Step 5: Run Trajectory Recorder
            print("="*70)
            print("📼 STEP 5: Recording Trajectories")
            print("="*70)

            recorder_cmd = f"source {self.setup_bash} && ros2 launch traj_recorder record_trajectory.launch.py " \
                          f"num_drones:={num_drones} output_folder:={output_folder} sampling_dt:={sampling_dt}"

            self.recorder_process = subprocess.Popen(
                recorder_cmd,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )

            # Start timeout timer
            timer_thread = threading.Thread(target=self.monitor_mission_completion, args=(timeout,))
            timer_thread.daemon = True
            timer_thread.start()

            # Monitor simulation output
            print("📡 Monitoring simulation...")
            print("   (Press Ctrl+C to stop manually)")
            print()

            while not self.mission_completed:
                if self.sim_process.poll() is not None:
                    print("⚠️  Simulation process ended")
                    break

                # Read simulation output
                line = self.sim_process.stdout.readline()
                if line:
                    # Check for mission completion
                    if "ALL DRONES ARRIVED" in line or "MISSION COMPLETE" in line:
                        print("✅ Mission completed successfully!")
                        self.mission_completed = True
                        self.stop_all_processes()
                        break

                time.sleep(0.1)

            # Wait for processes to finish
            if self.sim_process:
                self.sim_process.wait()
            if self.recorder_process:
                self.recorder_process.wait()

            # Find output folder
            print("\n" + "="*70)
            print("✅ PIPELINE COMPLETED")
            print("="*70)

            # Find generated trajectory folder
            parent_dir = Path.cwd()
            traj_folders = list(parent_dir.glob(f"{output_folder}_*"))

            if traj_folders:
                latest_folder = max(traj_folders, key=lambda p: p.stat().st_mtime)
                txt_files = list(latest_folder.glob("node_*.txt"))

                print(f"📁 Output folder: {latest_folder}")
                print(f"📄 Generated files: {len(txt_files)} trajectory files")
                print()

                if txt_files:
                    sample_file = txt_files[0]
                    print(f"📝 Sample file: {sample_file.name}")
                    print("   First 3 lines:")
                    with open(sample_file, 'r') as f:
                        for i, line in enumerate(f):
                            if i >= 3:
                                break
                            print(f"   {line.strip()}")

                print()
                print("="*70)
                print("🎉 SUCCESS! Trajectories generated.")
                print("="*70)
            else:
                print("⚠️  No trajectory folder found. Check recorder output.")

        except KeyboardInterrupt:
            print("\n\n🛑 Interrupted by user")
            self.stop_all_processes()

        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
            self.stop_all_processes()
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Fully Automated Formation Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 36 drones with preset formations
  python3 auto_formation_pipeline.py --num_drones 36

  # Custom formations
  python3 auto_formation_pipeline.py --current start.txt --target end.txt

  # With custom timeout
  python3 auto_formation_pipeline.py --num_drones 36 --timeout 90
        """
    )

    # Formation selection
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--num_drones', type=int, choices=[10, 25, 36],
                      help='Number of drones (preset formations: Grid→Triangle)')
    group.add_argument('--current', type=str,
                      help='Start formation file (x,y,z per line)')

    parser.add_argument('--target', type=str,
                       help='End formation file (required with --current)')

    # Output settings
    parser.add_argument('--output_folder', type=str, default='auto_trajectories',
                       help='Output folder name (default: auto_trajectories)')
    parser.add_argument('--sampling_dt', type=float, default=0.05,
                       help='Sampling interval in seconds (default: 0.05)')
    parser.add_argument('--timeout', type=int, default=120,
                       help='Auto-stop timeout in seconds (default: 120)')

    args = parser.parse_args()

    # Validation
    if args.current and not args.target:
        parser.error("--target is required when --current is specified")

    # Run pipeline
    pipeline = AutoFormationPipeline()

    pipeline.run_full_pipeline(
        num_drones=args.num_drones,
        current_file=args.current,
        target_file=args.target,
        output_folder=args.output_folder,
        sampling_dt=args.sampling_dt,
        timeout=args.timeout
    )


if __name__ == "__main__":
    main()
