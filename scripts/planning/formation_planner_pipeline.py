#!/usr/bin/env python3
"""
Formation Planner Pipeline
==========================
통합 프레임워크: 포메이션 입력 → 헝가리안 할당 → Ego-Swarm → 궤적 출력

Usage:
    python3 formation_planner_pipeline.py --num_drones 36
    python3 formation_planner_pipeline.py --current custom_formation_a.txt --target custom_formation_b.txt
"""

import os
import sys
import argparse
import subprocess
import time
import signal
from pathlib import Path

# Add parent scripts directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from planning.formations_large import get_formations, FORMATION_36_A, FORMATION_36_B
from Allocation.algorithms.Fair_Hungarian_Allocator import HungarianAllocator


class FormationPlannerPipeline:
    """통합 포메이션 플래닝 파이프라인"""

    def __init__(self, workspace_root=None):
        if workspace_root is None:
            # 스크립트 위치에서 workspace root 추정
            self.workspace_root = Path(__file__).parent.parent
        else:
            self.workspace_root = Path(workspace_root)

        self.scripts_dir = self.workspace_root / "scripts"
        self.src_dir = self.workspace_root / "src"

        print(f"Workspace root: {self.workspace_root}")

    def load_formation_from_file(self, filepath):
        """
        파일에서 포메이션 로드
        형식: x,y,z per line
        """
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
        """포메이션 로드 (하드코딩 또는 파일에서)"""

        if current_file and target_file:
            print(f"\n{'='*70}")
            print("Loading custom formations from files")
            print(f"{'='*70}")
            formation_a = self.load_formation_from_file(current_file)
            formation_b = self.load_formation_from_file(target_file)
            print(f"✓ Current formation: {len(formation_a)} drones from {current_file}")
            print(f"✓ Target formation: {len(formation_b)} drones from {target_file}")

            if len(formation_a) != len(formation_b):
                raise ValueError(f"Formation size mismatch: {len(formation_a)} vs {len(formation_b)}")

            return formation_a, formation_b, len(formation_a)

        elif num_drones:
            print(f"\n{'='*70}")
            print(f"Loading preset formations for {num_drones} drones")
            print(f"{'='*70}")
            formation_a, formation_b = get_formations(num_drones)
            print(f"✓ Current formation: Grid {len(formation_a)} drones")
            print(f"✓ Target formation: Triangle {len(formation_b)} drones")
            return formation_a, formation_b, num_drones

        else:
            raise ValueError("Must specify either num_drones or both current_file and target_file")

    def run_hungarian_allocation(self, formation_a, formation_b):
        """헝가리안 할당 실행"""
        print(f"\n{'='*70}")
        print("Running Fair Hungarian Allocation")
        print(f"{'='*70}")

        # Convert to allocator format [x, y, z]
        drones = [[pos[0], pos[1], pos[2]] for pos in formation_a]
        goals = [[pos[0], pos[1], pos[2]] for pos in formation_b]

        allocator = HungarianAllocator(max_iterations=50)
        assignment = allocator.allocate_fair(drones, goals)

        print(f"✓ Allocation completed")
        return assignment

    def generate_launch_file(self, formation_a, formation_b, assignment, num_drones, output_file):
        """동적으로 launch 파일 생성"""
        print(f"\n{'='*70}")
        print(f"Generating launch file: {output_file}")
        print(f"{'='*70}")

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

        # Launch 파일 내용 생성
        launch_content = f"""import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Map parameters
    map_size_x = LaunchConfiguration('map_size_x', default = 10.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 100.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 50.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap)

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)

    # Drone configurations - {num_drones} drones with Fair Hungarian Allocation
    drone_configs = [
"""

        # drone_configs 추가
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
    swarm_sync_script = os.path.join(project_root, 'scripts', 'core', 'swarm_synchronizer.py')

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

    mission_timer = ExecuteProcess(
        cmd=timer_cmd,
        output='screen'
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)

    # Add swarm synchronizer and mission timer
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)

    # Add drone nodes
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld
"""

        # 파일 쓰기
        with open(output_file, 'w') as f:
            f.write(launch_content)

        print(f"✓ Launch file generated: {output_file}")

    def run_simulation(self, launch_file, run_time=None):
        """Ego-Swarm 시뮬레이션 실행"""
        print(f"\n{'='*70}")
        print("Starting Ego-Swarm simulation")
        print(f"{'='*70}")
        print(f"Launch file: {launch_file}")
        if run_time:
            print(f"Auto-stop after: {run_time} seconds")
        print("\nPress Ctrl+C to stop simulation manually\n")

        cmd = ['ros2', 'launch', str(launch_file)]

        try:
            if run_time:
                # 제한 시간으로 실행
                process = subprocess.Popen(cmd)
                time.sleep(run_time)
                print(f"\n{'='*70}")
                print(f"Stopping simulation after {run_time} seconds")
                print(f"{'='*70}")
                process.send_signal(signal.SIGINT)
                process.wait()
            else:
                # 사용자가 종료할 때까지 실행
                subprocess.run(cmd)

        except KeyboardInterrupt:
            print("\n\nSimulation stopped by user")

    def run_trajectory_recorder(self, num_drones, output_folder, sampling_dt=0.05, run_time=None):
        """Trajectory recorder 실행"""
        print(f"\n{'='*70}")
        print("Starting trajectory recorder")
        print(f"{'='*70}")
        print(f"Number of drones: {num_drones}")
        print(f"Output folder: {output_folder}")
        print(f"Sampling dt: {sampling_dt}s")
        if run_time:
            print(f"Auto-stop after: {run_time} seconds")
        print("\nPress Ctrl+C to stop recording\n")

        cmd = [
            'ros2', 'launch', 'traj_recorder', 'record_trajectory.launch.py',
            f'num_drones:={num_drones}',
            f'output_folder:={output_folder}',
            f'sampling_dt:={sampling_dt}'
        ]

        try:
            if run_time:
                process = subprocess.Popen(cmd)
                time.sleep(run_time)
                print(f"\n{'='*70}")
                print(f"Stopping recorder after {run_time} seconds")
                print(f"{'='*70}")
                process.send_signal(signal.SIGINT)
                process.wait()
            else:
                subprocess.run(cmd)

        except KeyboardInterrupt:
            print("\n\nRecording stopped by user")

    def run_full_pipeline(self, num_drones=None, current_file=None, target_file=None,
                         output_folder="trajectories", sampling_dt=0.05, run_time=None):
        """전체 파이프라인 실행"""
        print(f"\n{'='*70}")
        print("Formation Planner Pipeline - Full Run")
        print(f"{'='*70}\n")

        # 1. 포메이션 로드
        formation_a, formation_b, num_drones = self.load_formations(
            num_drones, current_file, target_file)

        # 2. 헝가리안 할당
        assignment = self.run_hungarian_allocation(formation_a, formation_b)

        # 3. Launch 파일 생성
        launch_file = self.workspace_root / "src" / "planner" / "plan_manage" / "launch" / "generated_swarm.launch.py"
        self.generate_launch_file(formation_a, formation_b, assignment, num_drones, launch_file)

        # 4. 사용자에게 다음 단계 안내
        print(f"\n{'='*70}")
        print("Next Steps:")
        print(f"{'='*70}")
        print("\n1. Start simulation (Terminal 1):")
        print(f"   cd {self.workspace_root}")
        print(f"   source install/setup.bash")
        print(f"   ros2 launch {launch_file}")
        print("\n2. Start trajectory recorder (Terminal 2):")
        print(f"   cd {self.workspace_root}")
        print(f"   source install/setup.bash")
        print(f"   ros2 launch traj_recorder record_trajectory.launch.py \\")
        print(f"       num_drones:={num_drones} \\")
        print(f"       output_folder:={output_folder} \\")
        print(f"       sampling_dt:={sampling_dt}")
        print("\n3. Wait for simulation to complete, then press Ctrl+C")
        print("\n4. Check output folder: {output_folder}_YYYYMMDD_HHMMSS/")
        print(f"{'='*70}\n")

        return launch_file


def main():
    parser = argparse.ArgumentParser(description="Formation Planner Pipeline")

    # 포메이션 선택
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--num_drones', type=int, choices=[10, 25, 36],
                      help='Number of drones (use preset formations)')
    group.add_argument('--current', type=str,
                      help='Current formation file (x,y,z per line)')

    parser.add_argument('--target', type=str,
                       help='Target formation file (required if --current is used)')

    # 출력 설정
    parser.add_argument('--output_folder', type=str, default='trajectories',
                       help='Output folder name for trajectories')
    parser.add_argument('--sampling_dt', type=float, default=0.05,
                       help='Trajectory sampling interval (seconds)')

    # 실행 모드
    parser.add_argument('--mode', type=str, choices=['plan', 'sim', 'record', 'full'],
                       default='plan',
                       help='Execution mode: plan (generate only), sim (run simulation), record (record trajectories), full (all)')
    parser.add_argument('--run_time', type=float, default=None,
                       help='Auto-stop after N seconds (for sim/record modes)')

    args = parser.parse_args()

    # Validation
    if args.current and not args.target:
        parser.error("--target is required when --current is specified")

    # Run pipeline
    pipeline = FormationPlannerPipeline()

    try:
        if args.mode == 'plan':
            pipeline.run_full_pipeline(
                num_drones=args.num_drones,
                current_file=args.current,
                target_file=args.target,
                output_folder=args.output_folder,
                sampling_dt=args.sampling_dt
            )

        elif args.mode == 'full':
            # TODO: Implement full automated pipeline
            print("Full automated mode not yet implemented. Use 'plan' mode and follow instructions.")

        else:
            print(f"Mode '{args.mode}' not yet implemented")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
